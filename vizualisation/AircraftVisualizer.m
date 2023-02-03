classdef AircraftVisualizer
    % Based on https://github.com/bernhardpg/babyshark_vtol_model
    properties
        Model3D
        Animation
    end
    
    methods
        function obj = AircraftVisualizer()
            % Animation parameters %
            obj.Animation.timestep = 0.01;
            obj.Animation.zoom = 1.0;
            obj.Animation.show_position_plot = true;
            
            % 3D Model constants %
            obj.Model3D.cg_position_from_front = 0;
            obj.Model3D.cg_position_from_bottom = 0;
            obj.Model3D.wingspan = 0.4;
            obj.Model3D.alpha = 0.5;
            obj.Model3D.color = [0.85 0.85 0.9];
            
            % Initialize visualizer object %
            model = stlread('3d_files/CoaxialRobot.stl');
            obj.Model3D.stl_data.vertices = model.vertices;
            obj.Model3D.stl_data.faces = model.faces;
            obj = obj.initialize_aircraft_model();
            obj.Model3D.max_aircraft_dimensions = max(max(sqrt(sum(obj.Model3D.stl_data.vertices.^2, 2))));
            obj = obj.create_ax_object();
            obj.Animation.aircraft_transformation = hgtransform('Parent', obj.Animation.PlotAxes.ax_3d);
            obj.Animation.force_transformation = hgtransform('Parent', obj.Animation.PlotAxes.ax_3d);
            obj.Animation.force_magnitude = 10;
            obj.Animation.force_vector = patch;
        end
        
        function obj = initialize_aircraft_model(obj)
            V0 = obj.Model3D.stl_data.vertices;
            initial_phi = 0;
            initial_theta = 0;
            initial_psi = 0;
            %V0 = obj.rotate_vertices(V0, initial_phi, initial_theta, initial_psi);

            % Scale the aircraft 3d model to the correct size
            %V0 = obj.scale_aircraft(obj.Model3D.wingspan, V0);

            % Move origin to front of aircraft nose
            temp_max = max(V0);
            temp_min = min(V0);
            ranges = abs(temp_max - temp_min);
            aircraft_length = ranges(1);
            V0 = V0 - [aircraft_length obj.Model3D.wingspan/2 0];

            % Move model origin to correct aircraft cg
            cg_position = [...
                obj.Model3D.cg_position_from_front ...
                0 ...
                obj.Model3D.cg_position_from_bottom...
                ];
            %V0 = V0 - cg_position; 
            
            %obj.Model3D.stl_data.vertices = V0;
        end
        
        function V_scaled = scale_aircraft(~, wingspan, V)
            temp_max = max(V);
            temp_min = min(V);
            ranges = abs(temp_max - temp_min);
            y_range = ranges(2);
            zoom_factor = y_range / wingspan;
            V_scaled = V / zoom_factor;
        end
        
        function V_rotated = rotate_vertices(~, V, phi, theta, psi)
            Rx = [1 0 0;
                  0 cos(phi) -sin(phi);
                  0 sin(phi) cos(phi)];

            Ry = [cos(theta) 0 sin(theta);
                  0 1 0;
                  -sin(theta) 0 cos(theta)];

            Rz = [cos(psi), -sin(psi), 0 ;
                  sin(psi), cos(psi), 0 ;
                         0,         0, 1 ];

            V_rotated = V * Rx';
            V_rotated = V_rotated * Ry';
            V_rotated = V_rotated * Rz';
        end
        
        function render_plot(obj)
            axis(obj.Animation.PlotAxes.ax_3d, 'equal');
            viewbox = [-1 1 -1 1 -1 1] * (1/obj.Animation.zoom) * obj.Model3D.max_aircraft_dimensions;
            axis(obj.Animation.PlotAxes.ax_3d, viewbox);
            set(gcf,'Color',[1 1 1])
            
            view(obj.Animation.PlotAxes.ax_3d, [210 10])
            camlight(obj.Animation.PlotAxes.ax_3d, 'left');
            material(obj.Animation.PlotAxes.ax_3d, 'dull');
        end
        
        function plot_aircraft(obj)
            patch(obj.Animation.PlotAxes.ax_3d, 'Faces', obj.Model3D.stl_data.faces, ...
                'Vertices', obj.Model3D.stl_data.vertices, ...
                 'FaceColor', obj.Model3D.color, ...
                 'FaceAlpha', obj.Model3D.alpha, ...
                 'EdgeColor',       'none',        ...
                 'FaceLighting',    'gouraud',     ...
                 'AmbientStrength', 0.15,...
                 'Parent', obj.Animation.aircraft_transformation);
             % Plot body frame axes
             MakeFrame(zeros(3,1), [1 0 0;0 -1 0; 0 0 -1], 60, 10, 0.6, '', ...
                 'Parent', obj.Animation.aircraft_transformation, ...
                 'color', 'b');
             scatter3(obj.Animation.PlotAxes.ax_3d, 0,0,0,'filled');
             obj.render_plot();
        end
        
        function plot_external_forces_moments(obj)
            axes(obj.Animation.PlotAxes.ax_3d);
            MakeArrow(zeros(3,1), [5;0;0],  15, 0.05,'', 'color', 'g','Parent', obj.Animation.force_transformation);
            
        end
        function obj = plot_inputs(obj, t, x)
            y_datasources = ["roll_evolution" "pitch_evolution" "yaw_evolution"];
            input_plot_names = ["roll" "pitch", "yaw"];
            ylabels = ["deg" "deg" "edg"];
            
            for i = 1:3
                obj.Animation.InputPlotHandles{i} = plot(obj.Animation.PlotAxes.ax_inputs{i}, t(1), rad2deg(x(1)), 'XDataSource', 'time', 'YDataSource', y_datasources(i));
                xlim(obj.Animation.PlotAxes.ax_inputs{i}, [t(1), t(end)]);
                ylim(obj.Animation.PlotAxes.ax_inputs{i}, [-27, 27]);
                
                title(obj.Animation.PlotAxes.ax_inputs{i}, input_plot_names(i));
                ylabel(obj.Animation.PlotAxes.ax_inputs{i}, ylabels(i));
                grid(obj.Animation.PlotAxes.ax_inputs{i}, 'on'); 
                box(obj.Animation.PlotAxes.ax_inputs{i}, 'on');
            end
            
            xlabel(obj.Animation.PlotAxes.ax_inputs{3}, "Time [s]");
        end
        
        function [t, x] = interpolate_trajectory_for_animation(obj, t_trajectory, x_trajectory)
            t_0 = t_trajectory(1);
            t_end = t_trajectory(end);
            t = t_0:obj.Animation.timestep:t_end;
            x = interp1(t_trajectory,x_trajectory,t); 
        end
        
        function obj = plot_text(obj)
            obj.Animation.time_text_handle = text(...
                obj.Animation.PlotAxes.ax_3d, ...
                0 * obj.Model3D.max_aircraft_dimensions, ...
                0 * obj.Model3D.max_aircraft_dimensions, ...
                1 * obj.Model3D.max_aircraft_dimensions, ...
                't = 0 sec',...
                'FontSize', 20);
        end
        
        function obj = plot_position(obj, pos)
            obj.Animation.PosPlotHandle = plot3(obj.Animation.PlotAxes.ax_pos, pos(1,1), pos(1,2), pos(1,3), ...
            'XDataSource', 'pos_n', 'YDataSource', 'pos_w', 'ZDataSource', 'pos_h');
            axis(obj.Animation.PlotAxes.ax_pos, 'equal');
            mAnimation.PlotAxes.ax_pos = max([abs(pos); [2 2 2]]);
            viewbox = [-mAnimation.PlotAxes.ax_pos(1) mAnimation.PlotAxes.ax_pos(1) -mAnimation.PlotAxes.ax_pos(2) mAnimation.PlotAxes.ax_pos(2) -mAnimation.PlotAxes.ax_pos(3) mAnimation.PlotAxes.ax_pos(3)] * 1.2;
            axis(obj.Animation.PlotAxes.ax_pos, viewbox);
            set(gcf,'Color',[1 1 1])
            view(obj.Animation.PlotAxes.ax_pos, [30 10])
            grid(obj.Animation.PlotAxes.ax_pos, 'on');
            
            title(obj.Animation.PlotAxes.ax_pos, 'Position', 'FontSize', 16)
            xlabel(obj.Animation.PlotAxes.ax_pos, "north [m]");
            ylabel(obj.Animation.PlotAxes.ax_pos, "west [m]");
            zlabel(obj.Animation.PlotAxes.ax_pos, "altitude [m]");
        end
        
        function plot_trajectory(obj, t_trajectory, x_trajectory, u_trajectory)
            [t, x] = obj.interpolate_trajectory_for_animation(t_trajectory, x_trajectory);
            [~, u] = obj.interpolate_trajectory_for_animation(t_trajectory, u_trajectory);

            n = x(:,1);
            e = x(:,2);
            d = x(:,3);
            pos = [n -e -d];
            phi = x(:,10);
            theta = x(:,11);
            psi = x(:,12);
            
            F_x = u(:,1);
            F_y = u(:,2);
            F_z = u(:,3);
            
            obj.plot_aircraft();
            obj.plot_external_forces_moments()
            if obj.Animation.show_position_plot
                obj = obj.plot_position(pos);
            end
            obj = obj.plot_inputs(t, x);
            obj = obj.plot_text();
            % To let the display load for the user
            pause(1)
            tic;
            for i = 1:length(t)
                % Rotate the aircraft rigid-body to body frame
                Mx = makehgtform('xrotate', phi(i));
                My = makehgtform('yrotate', -theta(i));
                Mz = makehgtform('zrotate', -psi(i));
                set(obj.Animation.aircraft_transformation, 'Matrix', Mx*My*Mz);
                
                % Force vector is attached to body frame in positive
                % x-direction [1 0 0]
                
                % Find angle to rotate [1 0 0] in body frmae to force
                % in body frame
                force_rot = vrrotvec([1;0;0], [F_x(i);F_y(i);-F_z(i)]);
                Mr = makehgtform('axisrotate',force_rot(1:3),force_rot(4));
                % Scale by force magnitude
                Ms = makehgtform('scale', norm([F_x(i);F_y(i);F_z(i)]));
                % Do transformation
                set(obj.Animation.force_transformation, 'Matrix', Mx*My*Mz*Mr*Ms);
                
                % Update time text
                set(obj.Animation.time_text_handle, 'String', sprintf('t = %3.2f sec',t(i)))

                % Update position plot
                if obj.Animation.show_position_plot
                    pos_n = pos(1:i,1);
                    pos_w = pos(1:i,2);
                    pos_h = pos(1:i,3);
                    refreshdata(obj.Animation.PosPlotHandle, 'caller');
                end
                
                % Update input plots
                time = t(1:i);
                roll_evolution = rad2deg(x(1:i,9));
                pitch_evolution = rad2deg(x(1:i,10));
                yaw_evolution = rad2deg(x(1:i,11));
                refreshdata(obj.Animation.InputPlotHandles{1}, 'caller');
                refreshdata(obj.Animation.InputPlotHandles{2}, 'caller');
                refreshdata(obj.Animation.InputPlotHandles{3}, 'caller');
                
                % Control the animation speed to be realtime
                if obj.Animation.timestep * i - toc > 0
                    pause(max(0, obj.Animation.timestep * i - toc))
                end
            end
        end
        
        function obj = create_ax_object(obj)
            obj.Animation.fig = figure;
            screensize = get(0,'ScreenSize');
            
            % Show position plot and move other plots
            if obj.Animation.show_position_plot
                obj.Animation.PlotAxes.ax_pos = axes(obj.Animation.fig, 'position',[0.05 0.5 0.2 0.4]);
                
                pos_plot_position = [0.25 0.0 0.5 1];
                input_plot_positions = [0.75 0.7 0.2 0.15;
                                        0.75 0.5 0.2 0.15;
                                        0.75 0.3 0.2 0.15];
            else
                pos_plot_position = [0 0.0 0.5 1];
                input_plot_positions = [0.6 0.7 0.35 0.15;
                                        0.6 0.5 0.35 0.15;
                                        0.6 0.3 0.35 0.15];
            end
            
            set(gcf,'Position', ...
                [screensize(3)/40 screensize(4)/12 screensize(3)/1.5 screensize(4)/1.5],...
                'Visible','on');
            
            obj.Animation.PlotAxes.ax_3d = axes(obj.Animation.fig, 'position', pos_plot_position);
            axis off
            set(obj.Animation.PlotAxes.ax_3d,'color','none');
            axis(obj.Animation.PlotAxes.ax_3d, 'equal')
            hold on;
                                    
            for i = 1:3
                obj.Animation.PlotAxes.ax_inputs{i} = axes(obj.Animation.fig, 'position', input_plot_positions(i,:)); 
            end
        end
    end
end

