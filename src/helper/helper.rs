use ljni::JNIEnv;
use ljni::objects::JDoubleArray;
use ljni::sys::jdoubleArray;

pub fn jdoublearray_to_array(env: &JNIEnv, data: jdoubleArray) -> Vec<f64> {
    let data_d = unsafe { JDoubleArray::from_raw(data) };
    let len = env.get_array_length(&data_d).unwrap() as usize;
    let mut buf = vec![0f64; len];
    env.get_double_array_region(data_d, 0, &mut buf).unwrap();
    buf
}
