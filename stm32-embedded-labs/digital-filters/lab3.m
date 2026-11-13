% ===== Filter design (LPF/HPF) : fc=100 Hz, Fs=10 kHz =====
s  = tf('s');
fc = 100;                  % Hz
wc = 2*pi*fc;              % rad/s
w0 = wc;
Fs = 10000;                 % Hz
Ts = 1/Fs;
Q = 5;
tau = 1/wc;

LPF_s  = 1/(tau*s + 1);
HPF_s  = (tau*s)/(tau*s + 1);
APF_s  = (tau*s - 1)/(tau*s + 1);
BPF_s = ((w0/Q)*s) / (s^2 + (w0/Q)*s + w0^2);
BSF_s = (s^2 + w0^2) / (s^2 + (w0/Q)*s + w0^2);

% Tustin with prewarp at wc
opts = c2dOptions('Method','tustin','PrewarpFrequency',wc);

LPF_z  = c2d(LPF_s, Ts, opts);
HPF_z  = c2d(HPF_s, Ts, opts);
APF_z  = c2d(APF_s, Ts, opts);
BPF_z = c2d(BPF_s, Ts, opts);
BSF_z = c2d(BSF_s, Ts, opts);

% Coefficients for Discrete Transfer Fcn / C implementation
[numLPF, denLPF] = tfdata(LPF_z, 'v');   % numLPF = [b0 b1], denLPF=[1 a1]
[numHPF, denHPF] = tfdata(HPF_z, 'v');   % numHPF = [b0 b1], denHPF=[1 a1]
[numAPF, denAPF] = tfdata(APF_z, 'v');
[numBPF, denBPF] = tfdata(BPF_z, 'v');
[numBSF, denBSF] = tfdata(BSF_z, 'v');

% 편의변수(차분방정식용)
b0_l = numLPF(1); b1_l = numLPF(2); a1_l = -denLPF(2);  % y[n]=a1_l*y[n-1]+b0_l*x[n]+b1_l*x[n-1]
b0_h = numHPF(1); b1_h = numHPF(2); a1_h = -denHPF(2);
b0_a = numAPF(1); b1_a = numAPF(2); a1_a = -denAPF(2);
b0_bp = numBPF(1); b1_bp = numBPF(2); b2_bp = numBPF(3); a1_bp = -denBPF(2); a2_bp = -denBPF(3);
b0_bs = numBSF(1); b1_bs = numBSF(2); b2_bs = numBSF(3); a1_bs = -denBSF(2); a2_bs = -denBSF(3);
% 숫자 확인
% LPF_z, HPF_z, APF_z, numLPF, denLPF, numHPF, denHPF, numAPF, denAPF
BPF_s, BSF_s, BPF_z, BSF_z, numBPF, denBPF, numBSF, denBSF
