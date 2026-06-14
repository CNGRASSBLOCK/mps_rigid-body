package org.polaris2023.msp_rigid_body.ffm;

import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SymbolLookup;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.nio.file.Path;

public final class RigidBodyFfm {
    public static final MemoryLayout BOOL = MemoryLayout.structLayout(ValueLayout.JAVA_BYTE.withName("_0"));
    public static final MemoryLayout VEC3 = MemoryLayout.structLayout(
            ValueLayout.JAVA_DOUBLE.withName("x"),
            ValueLayout.JAVA_DOUBLE.withName("y"),
            ValueLayout.JAVA_DOUBLE.withName("z"));
    public static final MemoryLayout QUAT = MemoryLayout.structLayout(
            ValueLayout.JAVA_DOUBLE.withName("i"),
            ValueLayout.JAVA_DOUBLE.withName("j"),
            ValueLayout.JAVA_DOUBLE.withName("k"),
            ValueLayout.JAVA_DOUBLE.withName("w"));
    public static final MemoryLayout AABB = MemoryLayout.structLayout(
            VEC3.withName("mins"),
            VEC3.withName("maxs"));

    public static final long VEC3_X = VEC3.byteOffset(MemoryLayout.PathElement.groupElement("x"));
    public static final long VEC3_Y = VEC3.byteOffset(MemoryLayout.PathElement.groupElement("y"));
    public static final long VEC3_Z = VEC3.byteOffset(MemoryLayout.PathElement.groupElement("z"));

    private static final Linker LINKER = Linker.nativeLinker();

    private final SymbolLookup lookup;
    private final Arena arena;
    private final MethodHandle abiVersion;
    private final MethodHandle worldCreate;
    private final MethodHandle worldDestroy;
    private final MethodHandle worldStep;
    private final MethodHandle worldSetGravity;
    private final MethodHandle worldGetGravityOut;
    private final MethodHandle rigidBodyBuilderCreate;
    private final MethodHandle rigidBodyBuilderDestroy;
    private final MethodHandle rigidBodyBuilderSetTranslation;
    private final MethodHandle worldInsertRigidBody;
    private final MethodHandle rigidBodyGetTranslationOut;
    private final MethodHandle crbTreeCreate;
    private final MethodHandle crbTreeDestroy;
    private final MethodHandle crbTreeInsertFlag;
    private final MethodHandle crbTreeQueryAabbCount;

    public RigidBodyFfm(Path library, Arena arena) {
        this.lookup = SymbolLookup.libraryLookup(library, arena);
        this.arena = arena;
        abiVersion = downcall("abi_version", FunctionDescriptor.of(ValueLayout.JAVA_INT));
        worldCreate = downcall("world_create", FunctionDescriptor.of(ValueLayout.ADDRESS, VEC3));
        worldDestroy = downcall("world_destroy", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        worldStep = downcall("world_step", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_DOUBLE));
        worldSetGravity = downcall("world_set_gravity", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, VEC3));
        worldGetGravityOut = downcall("world_get_gravity_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.ADDRESS));
        rigidBodyBuilderCreate = downcall("rigid_body_builder_create", FunctionDescriptor.of(ValueLayout.ADDRESS, ValueLayout.JAVA_INT));
        rigidBodyBuilderDestroy = downcall("rigid_body_builder_destroy", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        rigidBodyBuilderSetTranslation = downcall("rigid_body_builder_set_translation", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, VEC3));
        worldInsertRigidBody = downcall("world_insert_rigid_body", FunctionDescriptor.of(ValueLayout.JAVA_LONG, ValueLayout.ADDRESS, ValueLayout.ADDRESS));
        rigidBodyGetTranslationOut = downcall("rigid_body_get_translation_out", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, ValueLayout.ADDRESS));
        crbTreeCreate = downcall("crb_tree_create", FunctionDescriptor.of(ValueLayout.ADDRESS));
        crbTreeDestroy = downcall("crb_tree_destroy", FunctionDescriptor.ofVoid(ValueLayout.ADDRESS));
        crbTreeInsertFlag = downcall("crb_tree_insert_flag", FunctionDescriptor.of(ValueLayout.JAVA_BYTE, ValueLayout.ADDRESS, ValueLayout.JAVA_LONG, AABB));
        crbTreeQueryAabbCount = downcall("crb_tree_query_aabb_count", FunctionDescriptor.of(ValueLayout.JAVA_INT, ValueLayout.ADDRESS, AABB));
    }

    public int abiVersion() {
        try {
            return (int) abiVersion.invokeExact();
        } catch (Throwable throwable) {
            throw callFailed("abi_version", throwable);
        }
    }

    public MemorySegment worldCreate(double gravityX, double gravityY, double gravityZ) {
        try {
            return (MemorySegment) worldCreate.invokeExact(vec3(gravityX, gravityY, gravityZ));
        } catch (Throwable throwable) {
            throw callFailed("world_create", throwable);
        }
    }

    public void worldDestroy(MemorySegment world) {
        try {
            worldDestroy.invokeExact(world);
        } catch (Throwable throwable) {
            throw callFailed("world_destroy", throwable);
        }
    }

    public void worldStep(MemorySegment world, double deltaSeconds) {
        try {
            worldStep.invokeExact(world, deltaSeconds);
        } catch (Throwable throwable) {
            throw callFailed("world_step", throwable);
        }
    }

    public void worldSetGravity(MemorySegment world, double x, double y, double z) {
        try {
            worldSetGravity.invokeExact(world, vec3(x, y, z));
        } catch (Throwable throwable) {
            throw callFailed("world_set_gravity", throwable);
        }
    }

    public MemorySegment worldGetGravity(MemorySegment world) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            worldGetGravityOut.invokeExact(world, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("world_get_gravity_out", throwable);
        }
    }

    public MemorySegment rigidBodyBuilderCreate(int status) {
        try {
            return (MemorySegment) rigidBodyBuilderCreate.invokeExact(status);
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_builder_create", throwable);
        }
    }

    public void rigidBodyBuilderDestroy(MemorySegment builder) {
        try {
            rigidBodyBuilderDestroy.invokeExact(builder);
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_builder_destroy", throwable);
        }
    }

    public void rigidBodyBuilderSetTranslation(MemorySegment builder, double x, double y, double z) {
        try {
            rigidBodyBuilderSetTranslation.invokeExact(builder, vec3(x, y, z));
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_builder_set_translation", throwable);
        }
    }

    public long worldInsertRigidBody(MemorySegment world, MemorySegment body) {
        try {
            return (long) worldInsertRigidBody.invokeExact(world, body);
        } catch (Throwable throwable) {
            throw callFailed("world_insert_rigid_body", throwable);
        }
    }

    public MemorySegment rigidBodyGetTranslation(MemorySegment world, long body) {
        MemorySegment out = arena.allocate(VEC3);
        try {
            rigidBodyGetTranslationOut.invokeExact(world, body, out);
            return out;
        } catch (Throwable throwable) {
            throw callFailed("rigid_body_get_translation_out", throwable);
        }
    }

    public MemorySegment crbTreeCreate() {
        try {
            return (MemorySegment) crbTreeCreate.invokeExact();
        } catch (Throwable throwable) {
            throw callFailed("crb_tree_create", throwable);
        }
    }

    public void crbTreeDestroy(MemorySegment tree) {
        try {
            crbTreeDestroy.invokeExact(tree);
        } catch (Throwable throwable) {
            throw callFailed("crb_tree_destroy", throwable);
        }
    }

    public boolean crbTreeInsert(MemorySegment tree, long id, MemorySegment aabb) {
        try {
            return ((byte) crbTreeInsertFlag.invokeExact(tree, id, aabb)) != 0;
        } catch (Throwable throwable) {
            throw callFailed("crb_tree_insert", throwable);
        }
    }

    public int crbTreeQueryAabbCount(MemorySegment tree, MemorySegment aabb) {
        try {
            return (int) crbTreeQueryAabbCount.invokeExact(tree, aabb);
        } catch (Throwable throwable) {
            throw callFailed("crb_tree_query_aabb_count", throwable);
        }
    }

    public MemorySegment vec3(double x, double y, double z) {
        MemorySegment value = arena.allocate(VEC3);
        value.set(ValueLayout.JAVA_DOUBLE, VEC3_X, x);
        value.set(ValueLayout.JAVA_DOUBLE, VEC3_Y, y);
        value.set(ValueLayout.JAVA_DOUBLE, VEC3_Z, z);
        return value;
    }

    public MemorySegment aabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        MemorySegment value = arena.allocate(AABB);
        value.asSlice(0, VEC3.byteSize()).copyFrom(vec3(minX, minY, minZ));
        value.asSlice(VEC3.byteSize(), VEC3.byteSize()).copyFrom(vec3(maxX, maxY, maxZ));
        return value;
    }

    public static double x(MemorySegment vec3) {
        return vec3.get(ValueLayout.JAVA_DOUBLE, VEC3_X);
    }

    public static double y(MemorySegment vec3) {
        return vec3.get(ValueLayout.JAVA_DOUBLE, VEC3_Y);
    }

    public static double z(MemorySegment vec3) {
        return vec3.get(ValueLayout.JAVA_DOUBLE, VEC3_Z);
    }

    private MethodHandle downcall(String symbol, FunctionDescriptor descriptor) {
        MemorySegment address = lookup.find(symbol)
                .orElseThrow(() -> new UnsatisfiedLinkError("missing native symbol: " + symbol));
        return LINKER.downcallHandle(address, descriptor);
    }

    private static IllegalStateException callFailed(String symbol, Throwable throwable) {
        return new IllegalStateException("native call failed: " + symbol, throwable);
    }
}
