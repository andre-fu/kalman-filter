# Basic Kalman Filtering 

To run:
1. Clone the repository 
2. Run the following code
```
cd kalman-filter
mkdir build
cd build
cmake ..
make
./kalman-filter
```
For Future Engineering Science Students: 
1. Record the circumference of your wheel
2. Calculate how fast you plan on setting the motor to go 
3. Calculate the expected position of your robot according to those parameters
4. Alter the `measurements` into `measureX` and `measureY`
5. Push the `measureX[i]` and `measureY[i]` into `z_measure`
6. Alter the Noise parameters according to your error bars on the motor (R matrix) 
7. Profit! 

<img src="https://render.githubusercontent.com/render/math?math=X = (x y \dot x \dot y )^T">

**Prediction Step**
<img src="https://render.githubusercontent.com/render/math?math=X_{k+1}' = F \cdot X_k + B*u">
<img src="https://render.githubusercontent.com/render/math?math=P_{k+1}' = F \cdot P_{k} \cdot F^T + Q">

**Update Step**
<img src="https://render.githubusercontent.com/render/math?math=K_k = P_k \cdot H^T (H \cdot P_k \cdot H^T + R )^{-1}">
<img src="https://render.githubusercontent.com/render/math?math=X_{k+1} = X_{k+1}' + K_k \cdot (z_k - H \cdot X_{k+1}')">
<img src="https://render.githubusercontent.com/render/math?math=P_k = (Id - K_k *H) * P_{k}'">



TODO:
Implement a way of getting the z, input measurements via stream
