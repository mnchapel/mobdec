#ifndef LDOF_GPU_H
#define LDOF_GPU_H

void ldof(CTensor<float> aImage1, CTensor<float> aImage2, CTensor<float>& aForward, CTensor<float>& aBackward);

void ldof(CTensor<float> aImage1, CTensor<float> aImage2, CTensor<float>& aForward, CTensor<float>& aBackward, float sigma, float alpha, float beta, float gamma, float eta, int fp_iterations, int linsolve_iterations);

void runFlowGPU(CTensor<float>& aImage1, CTensor<float>& aImage2, CTensor<float>& aResult, float sigma, float alpha, float beta, float gamma, float eta, int fp_iterations, int linsolve_iterations) ;

#endif /* LDOF_GPU_H */
