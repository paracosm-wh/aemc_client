# <center>不同匹配算法的核心思想与实现
## <center>1. 问题描述

##  <center>2. 算法对比 
### 2.1 ICP算法
### 2.2 KC算法
### 2.3 Wahba’s problem
 #### 2.3.1 问题定义
Wahba's problem: 仅通过 3D 旋转相差的两个点集的配准（即没有缩放和平移）[[1]](https://en.wikipedia.org/wiki/Wahba%27s_problem)。
其将问题定义为最小化代价函数：  
$J(\mathbf{R})=\frac{1}{2}\sum_{k=1}^{N}a_{k}\|\mathbf{w}_{k}-\mathbf{R}\mathbf{v}_{k}\|^{2}\text{for}N\geq2$
 #### 2.3.2 解决方法
##### a. 通过SVD分解求解
- $\mathbf{B}=\sum_{i=1}^na_i\mathbf{w}_i\mathbf{v}_i{}^T$
- 对$mathbf{B}$ 进行SVD分解：$B=USV^T$
- 旋转矩阵：$\mathbf{R}=UMV^T$ ，$M=diag(1,1,det(U)det(V))$