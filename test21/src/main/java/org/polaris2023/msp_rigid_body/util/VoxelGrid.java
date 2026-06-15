package org.polaris2023.msp_rigid_body.util;

public final class VoxelGrid implements AutoCloseable {
    private final int sizeX;
    private final int sizeY;
    private final int sizeZ;
    private final NativeMemory voxels;

    public VoxelGrid(int sizeX, int sizeY, int sizeZ) {
        if (sizeX <= 0 || sizeY <= 0 || sizeZ <= 0) {
            throw new IllegalArgumentException("voxel dimensions must be positive");
        }
        long count = (long) sizeX * sizeY * sizeZ;
        if (count > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("voxel grid is too large for Java helper");
        }
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        this.sizeZ = sizeZ;
        this.voxels = new NativeMemory(count);
    }

    public int sizeX() {
        return sizeX;
    }

    public int sizeY() {
        return sizeY;
    }

    public int sizeZ() {
        return sizeZ;
    }

    public long address() {
        return voxels.address();
    }

    public int count() {
        return Math.multiplyExact(Math.multiplyExact(sizeX, sizeY), sizeZ);
    }

    public byte[] toByteArray() {
        byte[] values = new byte[count()];
        long base = voxels.address();
        for (int i = 0; i < values.length; i++) {
            values[i] = NativeMemory.UNSAFE.getByte(base + i);
        }
        return values;
    }

    public VoxelGrid set(int x, int y, int z, boolean solid) {
        if (!contains(x, y, z)) {
            throw new IndexOutOfBoundsException("voxel coordinate is outside grid");
        }
        voxels.putByte(index(x, y, z), solid ? 1 : 0);
        return this;
    }

    public VoxelGrid fillBox(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
        int fromX = Math.max(0, minX);
        int fromY = Math.max(0, minY);
        int fromZ = Math.max(0, minZ);
        int toX = Math.min(sizeX, maxX);
        int toY = Math.min(sizeY, maxY);
        int toZ = Math.min(sizeZ, maxZ);
        for (int z = fromZ; z < toZ; z++) {
            for (int y = fromY; y < toY; y++) {
                for (int x = fromX; x < toX; x++) {
                    voxels.putByte(index(x, y, z), 1);
                }
            }
        }
        return this;
    }

    @Override
    public void close() {
        voxels.close();
    }

    private boolean contains(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < sizeX && y < sizeY && z < sizeZ;
    }

    private long index(int x, int y, int z) {
        return (long) z * sizeX * sizeY + (long) y * sizeX + x;
    }
}
