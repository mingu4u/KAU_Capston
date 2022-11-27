function [X,P]=linear_kalman_filter_function(X,P,Sdata,dt,Q,R,input)
    %%% Initialization %%%
      F = [1 -dt; 0 1];                     % 상태 공간 행렬
      B = [dt; 0];
      H = [1 0];                             % 관측 행렬 (구할 값)
    %%% Prediction stage%%%
    XPred = F*X+B*(input);                            
    PPred = F*P*F' + Q*dt;
    %%% Update stage %%%
    z = Sdata;                              % 실제 측정 값
    y = z - H*XPred ;                       % 측정값과 예측값 비교
    S = H*PPred*H' + R;                     % 예측 공분산 행렬
    K = PPred*H'/S;                         % 칼만 게인 업데이트
    XEst = XPred + K*y;                     % 최종 예측값
    PEst = (eye(size(X,1)) - K*H)*PPred;    % 공분산 
    %%% Update result %%%
    X = XEst;
    P = PEst;
end