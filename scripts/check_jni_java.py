import collections
import pathlib
import re
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
JNI = ROOT / "src" / "abi" / "jni.rs"
JAVA = (
    ROOT
    / "test21"
    / "src"
    / "main"
    / "java"
    / "org"
    / "polaris2023"
    / "msp_rigid_body"
    / "RigidBodyNative.java"
)


def main() -> int:
    jni = JNI.read_text(encoding="utf-8")
    java = JAVA.read_text(encoding="utf-8")
    rust_methods = re.findall(r"jni!\((?:\w+)\s+(\w+)\(", jni)
    java_methods = re.findall(r"native\s+\w+\s+(\w+)\(", java)

    missing = [method for method in rust_methods if method not in java_methods]
    duplicates = [
        method
        for method, count in collections.Counter(java_methods).items()
        if count > 1
    ]

    if missing:
        print("Missing Java native declarations:")
        print("\n".join(missing))
    if duplicates:
        print("Duplicate Java native declarations:")
        print("\n".join(duplicates))
    if missing or duplicates:
        return 1

    print(f"JNI declarations OK: {len(rust_methods)} Rust methods, {len(java_methods)} Java methods")
    return 0


if __name__ == "__main__":
    sys.exit(main())
