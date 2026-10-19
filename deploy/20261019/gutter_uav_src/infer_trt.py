#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO-seg TensorRT infer for Jetson (TRT7 + torch CUDA buffers, no pycuda)."""
from __future__ import print_function

import numpy as np


class YoloSegTRT(object):
    """Load .engine; input RGB letterbox HxWx3; return (det, proto) like RKNN."""

    def __init__(self, engine_path, imgsz=320):
        import tensorrt as trt
        import torch

        self.imgsz = int(imgsz)
        self.torch = torch
        logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(logger)
        with open(engine_path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError("deserialize failed: " + engine_path)
        self.context = self.engine.create_execution_context()
        self.bindings = [0] * self.engine.num_bindings
        self._in_buf = None
        self._out_meta = []  # (idx, shape, buf)

        for i in range(self.engine.num_bindings):
            name = self.engine.get_binding_name(i)
            shape = tuple(int(s) for s in self.engine.get_binding_shape(i))
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            tdtype = torch.float16 if dtype == np.float16 else torch.float32
            vol = 1
            for s in shape:
                vol *= s
            buf = torch.empty(vol, dtype=tdtype, device="cuda")
            self.bindings[i] = int(buf.data_ptr())
            is_in = self.engine.binding_is_input(i)
            print("[TRT]", i, name, shape, dtype, "IN" if is_in else "OUT")
            if is_in:
                self._in_buf = buf
                self._in_shape = shape
            else:
                self._out_meta.append((i, shape, buf))

        if self._in_buf is None or len(self._out_meta) < 2:
            raise RuntimeError("unexpected engine bindings")
        # warmup
        z = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        self.infer(z)

    def infer(self, rgb_u8):
        torch = self.torch
        if rgb_u8.shape[0] != self.imgsz or rgb_u8.shape[1] != self.imgsz:
            raise ValueError("need letterboxed %dx%d" % (self.imgsz, self.imgsz))
        x = rgb_u8.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        flat = np.ascontiguousarray(x.reshape(-1))
        self._in_buf.copy_(
            torch.from_numpy(flat).to(device="cuda", dtype=self._in_buf.dtype)
        )
        if not self.context.execute_v2(self.bindings):
            return None, None
        outs = []
        for _i, shape, buf in sorted(self._out_meta, key=lambda t: t[0]):
            outs.append(buf.detach().float().cpu().numpy().reshape(shape))
        # 按形状识别：det=(1,37,N) proto=(1,32,H,W)；binding 顺序可能颠倒
        det, proto = None, None
        for arr in outs:
            if arr is None:
                continue
            if arr.ndim == 3 and arr.shape[1] >= 37:
                det = arr
            elif arr.ndim == 4 and arr.shape[1] == 32:
                proto = arr
            elif arr.ndim == 3 and arr.shape[1] == 32:
                proto = arr
        if det is None or proto is None:
            # fallback 原顺序
            if len(outs) >= 2:
                return outs[0], outs[1]
            return None, None
        return det, proto


def build_engine_from_onnx(onnx_path, engine_path, fp16=True, workspace_mb=1024):
    import tensorrt as trt

    logger = trt.Logger(trt.Logger.INFO)
    flag = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    builder = trt.Builder(logger)
    network = builder.create_network(flag)
    parser = trt.OnnxParser(network, logger)
    builder.max_workspace_size = int(workspace_mb) * (1 << 20)
    if fp16 and builder.platform_has_fast_fp16:
        builder.fp16_mode = True
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(parser.get_error(i))
            raise RuntimeError("ONNX parse failed")
    engine = builder.build_cuda_engine(network)
    if engine is None:
        raise RuntimeError("build engine failed")
    with open(engine_path, "wb") as f:
        f.write(engine.serialize())
    print("[TRT] saved", engine_path)
    return engine_path


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 4 and sys.argv[1] == "build":
        build_engine_from_onnx(sys.argv[2], sys.argv[3])
    else:
        print("usage: python3 infer_trt.py build in.onnx out.engine")
