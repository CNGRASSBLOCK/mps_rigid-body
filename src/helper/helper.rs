use ljni::JNIEnv;
use ljni::objects::{JBooleanArray, JDoubleArray, JLongArray};
use ljni::sys::{jdoubleArray, jint, jlong, jlongArray, jsize};
use rapier3d::geometry::Array2;
use rapier3d::math::Vector;
use rapier3d::prelude::{ColliderBuilder};
use crate::ColliderBuilderHandle;

fn to_jlong<T>(value: *mut T) -> jlong {
    value as isize as jlong
}

pub fn array_to_array2(data: Vec<f64>, x: u32, y: u32) -> Array2<f64> {
    let mut data_array2 = Array2::<f64>::zeros(x as usize, y as usize);
    for xp in 0..x {
        for yp in 0..y {
            data_array2[(xp as usize, yp as usize)] = data[(yp * y + xp) as usize]
        }
    }
    data_array2
}

pub fn jdoublearray_to_array(env: &JNIEnv, data: jdoubleArray) -> Vec<f64> {
    let data_d = unsafe { JDoubleArray::from_raw(data) };
    let len = env.get_array_length(&data_d).unwrap() as usize;
    let mut buf = vec![0f64; len];
    env.get_double_array_region(data_d, 0, &mut buf).unwrap();
    buf
}