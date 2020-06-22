A= [1 1.5
    0 0.5];
B= [0.5
    1];
C = [1 0];
N = 6;
[F,G] = predict_mats(A,B,N);
F
G