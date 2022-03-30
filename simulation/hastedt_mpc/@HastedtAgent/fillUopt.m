%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Model Predictive Flocking with Obstacle Avoidance and Asymmetric Interaction Forces"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function fillUopt(obj,U,messages)
Tf_U = [];
for i = 1:obj.Hp
    Tf_U = [Tf_U;kron(kron(eye(obj.num_N+1), obj.e_nm(obj.Hp,i)), eye(obj.m))];
end
U_ = reshape(Tf_U\U,[obj.m*obj.Hp, (obj.num_N+1)]);
U_opt_ = reshape(obj.U_opt,[obj.m*obj.Hp, obj.numberAgents]);
U_opt_(:,obj.id) = U_(:,1);
obj.U_opt(1:obj.m*obj.Hp) = U_(:,1);
messageCounter = 1;
for message = messages
    neighborID = message.data.id;
    U_opt_(:,neighborID) = U_(:,messageCounter+1);
    messageCounter = messageCounter + 1;
end
obj.U_opt = reshape(U_opt_,[numel(U_opt_), 1]);
end