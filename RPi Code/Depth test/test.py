import depthai as dai
print(f"Version: {dai.__version__}")
print(f"Has createXLinkOut: {hasattr(dai.Pipeline, 'createXLinkOut')}")
print(f"Has XLinkOut in node: {hasattr(dai.node, 'XLinkOut')}")
print(f"Nodes available: {[n for n in dir(dai.node) if 'XLink' in n]}")
print(f"Root available: {[n for n in dir(dai) if 'XLink' in n]}")