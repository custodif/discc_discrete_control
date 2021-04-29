% Script made by Fernanda Custodio Pereira do Carmo
% 
% April 18, 2021
%
% Regularize GAN with discrete control
%

clear;

eta = 0.1;

lambda = 20.1;

feedback_control = 'y';
step_impulse = 's';

ts = 1;
model =1;
c = 1;
step_time = 500;

%run_clc_discc = 'clc  ';
run_clc_discc = 'both ';
%run_clc_discc = 'discc';



%----------------------- continuous -------------------%


figure;
hold all;

if  or (run_clc_discc == 'clc  ', run_clc_discc == 'both ')
    switch model


        case 1

            h_d_cont = tf([1 0],[1 0 1]);
            h_g_cont = tf([1],[1 0 1]);
            disp(model)

        case 4

            h_d_cont = tf([2 0],[4 2 1]);
            h_g_cont = tf([1],[4 2 1]);
            disp(model)

        case 5

            h_d_cont = tf([1 0],[1 4 1]);
            h_g_cont = tf([1],[1 4 1]);

            disp(model)
    end


    switch model

        case {2, 3}
            legend({'discriminator ','generator '},'Location','northeast')

        case {1, 4, 5}    

            T_D_clc = c2d(h_d_cont,eta);
            H_D_clc = feedback(T_D_clc, lambda);

            T_G_clc = c2d(h_g_cont,eta);
            H_G_clc = feedback(T_G_clc, lambda);


            if feedback_control == 'y'
                [num_D_clc, den_D_clc, ts_D_clc] = tfdata(H_D_clc, 'v');
                [num_G_clc, den_G_clc, ts_G_clc] = tfdata(H_G_clc, 'v');

            else    
                [num_D_clc, den_D_clc, ts_D_clc] = tfdata(T_D_clc, 'v');
                [num_G_clc, den_G_clc, ts_G_clc] = tfdata(T_G_clc, 'v');

            end

            [A_D_clc, B_D_clc, C_D_clc, D_D_clc] = tf2ss(num_D_clc,den_D_clc);
            [A_G_clc, B_G_clc, C_G_clc, D_G_clc] = tf2ss(num_G_clc,den_D_clc);

            sys_D_clc = ss(A_D_clc, B_D_clc, C_D_clc, D_D_clc, -1);
            sys_G_clc = ss(A_G_clc, B_G_clc, C_G_clc, D_G_clc, -1);

            if step_impulse == 's'
                y1 = step(sys_D_clc, 0:step_time);
                y2 = step(sys_G_clc, 0:step_time);
            else
                y1 = impulse(sys_D_clc, 0:step_time);
                y2 = impulse(sys_G_clc, 0:step_time);     
            end


            stem(0:step_time, y1, ':x', 'LineStyle','none',...
                     'MarkerFaceColor','none',...
                     'MarkerEdgeColor','red')
            stem(0:step_time, y2, ':x', 'LineStyle','none',...
                     'MarkerFaceColor','none',...
                     'MarkerEdgeColor','black')


            switch model

                case 1
                   legend({'discriminator - clc-WGAN ','generator - clc-WGAN'},...
                        'Location','northeast','NumColumns',2,'FontSize',11)
                case 4

                   legend({'discriminator - clc-vanilla GAN','generator - clc-vanilla GAN'},...
                        'Location','east','NumColumns',2,'FontSize',11)
                case 5

                   legend({'discriminator - clc-LSGAN','generator - clc-LSGAN'},...
                        'Location','east','NumColumns',2,'FontSize',11)

            end
            ylabel('phi or theta')
            xlabel('steps') 



    end

end
    
% smooth bi-linear game



if  or (run_clc_discc == 'discc', run_clc_discc == 'both ')

    switch model

        case 1
            J_dd = 0;
            J_gd = -1;
            J_dg = 1;
            J_gg = 0;

            A = [1 + eta*J_dd, eta*J_gd; eta*J_dg, 1 + eta*J_gg];

            B = [-eta*J_gd*c;-eta*J_gg*c];
            


        case 2
            J_dd = 0;
            J_gd = -1;
            J_dg = 1;
            J_gg = 0;

            A = [1, -eta; eta, 1-eta*eta];

            B = [-eta*J_gd*c;-eta*J_gg*c];


        case 3
            J_dd = 0;
            J_gd = -1;
            J_dg = 1;
            J_gg = 0;

            A = [1-eta*eta, -eta*(1-eta*eta); eta, 1-eta*eta];

            B = [-eta*J_gd*c;-eta*J_gg*c];



        case 4        
            J_dd = -1/2;
            J_gd = -1/2;
            J_dg = 1/2;
            J_gg = 0;

            A = [1 + eta*J_dd, eta*J_gd; eta*J_dg, 1 + eta*J_gg];

            B = [-eta*J_gd*c;-eta*J_gg*c];

        case 5
            J_dd = -4;
            J_gd = -1;
            J_dg = 1;
            J_gg = 0;

            A = [1 + eta*J_dd, eta*J_gd; eta*J_dg, 1 + eta*J_gg];

            B = [-eta*J_gd*c;-eta*J_gg*c];

    end


    C = [1 0; 0 1];
    D = [0; 0];

    sys = ss(A, B, C, D, -1);

    [num, den] = ss2tf(sys.A, sys.B, sys.C, sys.D);

    T_D = tf(num(1,:), den(1,:), ts);
    T_G = tf(num(2,:), den(1,:), ts);


    % % 
    if feedback_control == 'y'
        H_D = feedback(T_D, lambda);
    else
        H_D = T_D;
    end

    %H_D = T_D;
    % 
    %H_G = feedback(T_G, lambda)
    H_G = T_G;

    [num_D, den_D, ts_D] = tfdata(H_D, 'v');
    [num_G, den_G, ts_G] = tfdata(H_G, 'v');

    %[A_G, B_G, C_G, D_G] = tf2ss([num_G; num_D],den_D);

    [A_D, B_D, C_D, D_D] = tf2ss(num_D,den_D);
    [A_G, B_G, C_G, D_G] = tf2ss(num_G,den_D);


    sys_D = ss(A_D, B_D, C_D, D_D, -1);
    sys_G = ss(A_D, B_G, C_G, D_G, -1);



    if step_impulse == 's'
       y1 = step(sys_D, 0:step_time);
       y2 = step(sys_G, 0:step_time);
    else
       y1 = impulse(sys_D, 0:step_time);
       y2 = impulse(sys_G, 0:step_time);     
    end

    stem(0:step_time, y1, 'LineStyle','none',...
         'MarkerFaceColor','none',...
         'MarkerEdgeColor','green')
    stem(0:step_time, y2, 'LineStyle','none',...
         'MarkerFaceColor','none',...
         'MarkerEdgeColor','blue')

    switch model

        case 1
            if  run_clc_discc == 'discc'
                legend({'discriminator - discc-WGAN ','generator - discc-WGAN'},...
                'Location','northeast','NumColumns',2,'FontSize',11)
            else
                legend({'discriminator - clc-WGAN ','generator - clc-WGAN'...
                'discriminator - discc-WGAN ','generator - discc-WGAN'},...
                'Location','northeast','NumColumns',2,'FontSize',11)
            end
        case 4
            if  run_clc_discc == 'discc'
                legend({'discriminator - discc-vanilla GAN','generator - discc-vanilla GAN'},...
                'Location','east','NumColumns',2,'FontSize',11)
            else
                legend({'discriminator - clc-vanilla GAN ','generator - clc-vanilla GAN'...
                'discriminator - discc-vanilla GAN','generator - discc-vanilla GAN'},...
                'Location','east','NumColumns',2,'FontSize',11)
            end
        case 5
            if  run_clc_discc == 'discc'
                legend({'discriminator - discc-LSGAN','generator - discc-LSGAN'},...
                'Location','east','NumColumns',2,'FontSize',11)
            else
                legend({'discriminator - clc-LSGAN ','generator - clc-LSGAN'...
                'discriminator - discc-LSGAN','generator - discc-LSGAN'},...
                'Location','east','NumColumns',2,'FontSize',11)
            end
    end

    ylabel('phi or theta')
    xlabel('steps') 

end

hold off;

% % 
% k = (-25:0.1:25);
% r = rlocus(T_D,k);
% size(r)

% 
% k = (-25:0.001:25);
% r = nyquist(T_D,k);
% size(r)

% rlocus(T_D)
% nyquist(T_D*lambda)


