function [Str_sim, Fz_sim, CompData] = Fahrtsimulation6_3(Str, FZ, const)
% V6.3
% Anpassung für die genauere Berechnung der Leistung
% Änderung damit json und eigenes Format zur Berechnung verwendet werden können.
% Anpassungen der Variablennamen, damit auch die json-Dateien verwendet werden können
% Es werden die Parameter von den Json-Dateien übernommen.
% Es wurde eine Rampe für das Beschleunigen und Bremsen der Fahrzeuge implementiert.
mat = zeros(20000,1);
% Erzeugen von leeren Matrizen um die Daten zu schreiben (ohne bereits vorgegebene Felder dauert die Rechenzeit länger
x = mat;  a = mat; v = mat; s = mat; x_break = mat; x_rest = mat; F_break = mat; F_A = mat; W_run = mat; W_st = mat; W_a = mat; W_curve = mat;
W_tot = mat; mu = mat; F_wheel = mat; P_r = mat; E = mat; A = mat; maxSpeed = mat; r = mat; v_break = mat; x_break_2 = mat; x_break_3 = mat; v_break_4 = mat;
Break = mat;

% FZ.mass = 2 * FZ.mass ; % Wenn zweifache Belegung benötigt wird.
if(isfield(FZ, 'M_axle'))
    G_T = FZ.M_axle * FZ.N_axles * const.g; % Gewicht, welches durch die angetriebenen Achsen übertragen werden kann.
else
    G_T = 19.5 * 4 * const.g;
end

if(isfield(FZ, 'mu_break') == false)
    FZ.mu_break = 0.15;
end

if(isequal(FZ.maxAcceleration,[]))
    FZ.maxAcceleration = 0.8;
end

    for n = 1:200000
        if (n == 1)
            % Bestimmung der initialen Bedinungen
            v(n) = 0;
            a(n) = const.accRamp;
            x(n) = 0;
            x_rest(n) = Str.stations(end) - Str.stations(1) - x(n);
            F_A(n) = FZ.maxTractionForce;
            STOP = false;
        else
           % aktuelle Geschwindigkeit mit Beschleunigung des vorherigen Zeitschrittes über die Zeitdifferenz
            v(n) = a(n-1)*((n-(n-1))/10) * 3.6 + v(n-1);

            % zurückgelegte und restliche Strecke berechnen
            x(n) = ((n - (n-1))/10) * v(n)/3.6 + x(n-1);
            x_rest(n) = Str.stations(end) - Str.stations(1) - x(n);

             % Steigung des Streckenabschnittes bestimmen
             % s(n) = 14;
            for st = 1:length(Str.gradients)                 
                if (st == length(Str.gradients))
                    s(n) = Str.gradients(st, 2);
                    break 
                elseif (Str.gradients(st + 1, 1) >= x(n))
                    s(n) = Str.gradients(st, 2);
                    break
                end
            end
            % aktuelle Geschwindigkeitslimite bestimmen [km/h]
            % maxSpeed(n) = 75;
            for Limit = 1:length(Str.speedLimits)
                if (Limit == length(Str.speedLimits))
                    maxSpeed(n) = Str.speedLimits(Limit, 2);
                    break
                elseif (Str.speedLimits(Limit + 1, 1) >= x(n))
                    maxSpeed(n) = Str.speedLimits(Limit, 2);
                    break
                end
            end

            % Aktuelle Zugkraft berechnen
%             if (v(n) < FZ.v_start)
            if (FZ.maxTractionForce * v(n)/3.6 <= FZ.maxTractionPower)
                F_A(n) = FZ.maxTractionForce;
            else
                F_A(n) = FZ.maxTractionPower/(v(n)/3.6);
            end

            % Reibungskoeffizient berechnen
            mu(n) = 7.5/(v(n) + 44) + 0.161;

            % maximal übertragbare Kraft aufgrund von Adhäsion
            
            F_wheel(n) = mu(n) * G_T;

            % Laufwiderstand berechnen:
            if(isfield(FZ,'N_waggon'))
                W_run(n) = (1.9 + 0.0025 * v(n) + 4.8 * (FZ.N_waggon + 2.7)/(FZ.mass * const.g) * 0.0145 * (v(n) + 15)^2) * FZ.mass * const.g / 1000; % [kN]
            else
                W_run(n) = (FZ.rollingResistanceR0 + v(n) * FZ.rollingResistanceR1 + (v(n))^2 * FZ.rollingResistanceR2)/1000;
            end

            % Kurvenwiderstand berechnen:
            if(isfield(Str,'curvatures'))
                for c = 1:length(Str.curvatures)
                    if (c == length(Str.curvatures))
                        r(n) = (Str.curvatures(c,2) + Str.curvatures(c,3))/2;
                        break
                    elseif (Str.curvatures(c + 1, 1) >= x(n))
                        r(n) = (Str.curvatures(c,2) + Str.curvatures(c,3))/2;
                        break
                    end
                end
                if (r(n) < 300)
                    W_curve(n) = (FZ.mass * const.g * 500/(r(n)-55))/1000;
                else
                    W_curve(n) = (FZ.mass * const.g * 500/(r(n)-30))/1000;
                end
            else
                W_curve(n) = 0;
            end

            % Steigungswiderstand berechnen:
            W_st(n) = FZ.mass * const.g * s(n)/1000; % [kN]

            % Berechnung der maximalen Bremsstrecke mit maximaler Verzögerung
%             x_break(n) = abs(((v(n)/3.6).^2 - (v(1)/3.6).^2)/(2 * FZ.maxDeceleration));

            % Berechnung der Geschwindigkeit, ab welcher die
            % Bremsbeschleunigung reduziert werden muss
            c = (0:const.accRamp:abs(a(n-1)))';
            v_break_2 = 0;

            for i = 1:length(c)
                v_break_2 = (v_break_2/3.6 + c(i) * const.dt) * 3.6;
                x_break_2(n) = x_break_2(n) + v_break_2/3.6 * const.dt;
            end

            % Berechnung der Bremsstrecke
            v_break(n) = v(n);
            a0 = a(n-1);
            while (v_break(n) > 0)

                x_break(n) = x_break(n) + v_break(n)/3.6 * const.dt;                

                if (a0 <= FZ.maxDeceleration)
                    a0 = FZ.maxDeceleration;
                elseif (FZ.maxDeceleration < a0)
                    a0 = a0 - const.accRamp;
                end
                v_break(n) = (v_break(n)/3.6 + a0 * const.dt) * 3.6;

            end

            v_break_3 = v(n);
            x_break_3(n) = 0;
            c = flip(c);
            for i = 1:length(c)
                v_break_3 = (v_break_3/3.6 - c(i) * const.dt)*3.6;
                x_break_3(n) = x_break_3(n) + v_break_3/3.6 * const.dt;
            end

            % Berechnung der Geschwindigkeit, ab welcher wieder runterbeschleunigt werden muss, damit bei max. Geschw. eine
            % Beschleunigung von 0 erreicht wird
            v_break_4(n) = maxSpeed(n);
            % v_break_4(n) = Str.maxSpeed;
            if (a(n-1) > 0)     
                Break(n) = 1;
                for i = (0:const.accRamp:abs(a(n-1)))

                    v_break_4(n) = ((v_break_4(n))/3.6 - i * const.dt)*3.6;
                    
                end
            end
            
            % Entscheidung ob gebremst oder beschleunigt werden muss
            if (x_break(n) + x_break_3(n) + x_break_2(n) >= x_rest(n)) % 5
                STOP = true;
            elseif (x_break(n) < x_rest(n))
                STOP = false;
            end

            % Bremsen oder Beschleunigen
            if(STOP == true) % Wenn Bremsstrecke grösser oder gleich ist wie die Reststrecke muss gebremst werden. 
                % Berechnung der Bremskraft des Fahrzeugs: F_break
                F_break(n) = -FZ.mu_break * FZ.mass * const.g * cos(s(n)/1000);
                % Wenn die minimale Geschwindigkeit für das Reduzieren der
                % Beschleunigung erreicht ist
                if (v_break_2 + 0.3 >= v(n))
                    a(n) = a(n-1) + const.accRamp;
                else
                    % Wenn Geschwindigkeit grösser als 0 ist darf gebremst werden.
                    if(v(n) > 0 && (FZ.maxDeceleration * ((n-(n-1))/10) * 3.6 + v(n)) > 0)
                        if (F_break(n)/FZ.mass + const.g * sin(s(n)/1000) < FZ.maxDeceleration)
                            % Wenn maximale Beschleunigung grösser als maximal mögliche, dann abbremsen mit maximaler Bremskraft
                            if (a(n-1) - const.accRamp <= FZ.maxDeceleration)
                                a(n) = FZ.maxDeceleration;
                            else
                                % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                                a(n) = a(n-1) - const.accRamp;
                            end
                        else
                            if (a(n-1) - const.accRamp <= F_break(n)/FZ.mass + const.g * sin(s(n)/1000))
                                % Wenn Beschleunigung aufgrund der Bremskraft kleiner als max. Bremsverzögerung ist.
                                a(n) = F_break(n)/FZ.mass + const.g * sin(s(n)/1000);
                            else
                                % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                                a(n) = a(n-1) - const.accRamp;
                            end
                        end                
                    elseif((FZ.maxDeceleration * ((n-(n-1))/10) * 3.6 + v(n)) < 0 && v(n) > 0)
                        % Berechnung der Beschl. wenn mit max. Beschleunigung negative Geschw. erreicht wird und Geschw. > 0 ist.
                        if (a(n-1) - const.accRamp <= -v(n)/(((n - (n-1))/10)*3.6))
                            a(n) = -v(n)/(((n - (n-1))/10)*3.6);
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) - const.accRamp;
                        end
                    else % Wenn Geschwindigkeit 0 oder kleiner als 0 ist darf nicht mehr gebremst werden.
                        a(n) = 0;
                    end
                end
            elseif (v(n) > v_break_4(n) && Break(n) == 1)
                % Bei Gesch. v_break_4 muss langsam runterbeschleunigt
                % werden
                a(n) = a(n-1) - const.accRamp;
            % Wenn restliche Strecke grösser als x Meter ist darf Beschleunigt werden.x_rest(1,n) > 2
            elseif (x_break(n) < x_rest(n) && STOP == false)
                % Berechnung der neuen möglichen Beschleunigung wenn noch genügend weit weg vom Ziel
                if (v(n) > maxSpeed(n) + 0.1)
                    % Wenn Maximalgeschwindigkeit überschritten ist muss gebremst werden.
                    a(n) = a(n-1) - const.accRamp;
                elseif(W_tot(n-1) > F_A(n-1))
                    a(n) = a(n-1) - const.accRamp;
                elseif ((F_A(n) - (W_run(n) + W_st(n) + W_curve(n))) > 0 && F_wheel(n) >= (W_run(n) + W_st(n) + W_curve(n)))  

                    if (((F_A(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)) > FZ.maxAcceleration && v(n) < maxSpeed(n) && F_wheel(n) > F_A(n))
                        if (a(n-1) >= FZ.maxAcceleration - const.accRamp)
                            % max. Beschleunigung wenn genügend Kraft zur Verfügung steht und Maximalgeschw. nicht erreicht ist
                            a(n) = FZ.maxAcceleration;
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) + const.accRamp;
                        end

                    elseif (v(n) < maxSpeed(n) && F_wheel(n) < F_A(n) && ((F_wheel(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)) > FZ.maxAcceleration)
                        % max. Beschleunigung wenn Kraft auf Gleise kleiner als Zugkraft, diese jedoch genügend gross um Max.
                        % Beschleunigung zu erreichen. Geschw. < als max. Geschwindigkeit
                        if (a(n-1) >= FZ.maxAcceleration - const.accRamp)
                            % max. Beschleunigung wenn genügend Kraft zur Verfügung steht und Maximalgeschw. nicht erreicht ist
                            a(n) = FZ.maxAcceleration;
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) + const.accRamp;
                        end

                    elseif (v(n) < maxSpeed(n) && F_wheel(n) < F_A(n) && ((F_wheel(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)) < FZ.maxAcceleration)
                        % Beschleunigung wird berechnet über max. übertragbare Kraft auf Gleise, wenn diese kleiner
                        % ist als die Zugkraft. Geschw. < max. Geschw.
                        if(a(n-1) + const.accRamp >= ((F_wheel(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)))
                            a(n) = ((F_wheel(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho));
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) + const.accRamp;
                        end

                    elseif (v(n) < maxSpeed(n))
                        % Wenn Beschleunigung begrenzt wird durch zur Verfügung stehende Kraft wird sie berechnet
                        if (a(n-1) + const.accRamp >= ((F_A(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)))
                            a(n) = ((F_A(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho));
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) + const.accRamp;
                        end

                    else
                        % Wenn aktuelle Geschw. > max. Geschw.
                        if(a(n-1) > const.accRamp)
                            a(n) = a(n-1)-const.accRamp;
                        else
                            a(n) = 0;
                        end
                    end

                else
                    if(a(n-1) > const.accRamp)
                        a(n) = a(n-1)-const.accRamp;
                    else
                        a(n) = 0;
                    end
                end
            else
                if(abs(a(n-1)) > const.accRamp)
                    a(n) = a(n-1) + const.accRamp;
                else
                    a(n) = 0;
                end
            end

            % Beschleunigungswiderstand berechnen:
            W_a(n) = FZ.mass * a(n) * FZ.rho; % [kN]

            % Gesamtwiderstand berechnen:
            W_tot(n) = W_run(n) + W_st(n) + W_a(n) + W_curve(n); % [kN]

            % Berechnung der Leistung während der Fahrt (negative Leistung
            % beim Bremsen wird Rekuperiert. Bei v < 5 km/h wird nur pneumatische Bremse eingesetzt
            if (W_tot(n) * v(n)/3.6 >= 0)
                % Leistung die gebraucht wird für Traktion
                P_r(n) = ((W_tot(n) * v(n)/3.6) + const.AuxP)/const.nPantograph;
            elseif (W_tot(n) * v(n)/3.6 < 0 && v(n) > FZ.minSpeedRegBraking)
                if(abs((W_tot(n) * v(n)/3.6 + const.AuxP) * const.nPantograph) <= FZ.maxRegPower)
                    % Regenerative, negative Leistung
                    P_r(n) = (W_tot(n) * v(n)/3.6 + const.AuxP) * const.nPantograph;
                else
                    P_r(n) = -FZ.maxRegPower;
                end
            else
                P_r(n) = (0 + const.AuxP) * const.nPantograph; 
            end

            % P_r(n) = 2 * P_r(n); % während der Woche fahren die Züge doppelt

            % Berechnung der während der Fahrt benötigten Energie
            E(n) = P_r(n) * const.dt;

            % Berechnung der geleisteten Traktionsarbeit (sollte gleich sein wie E)
            A(n) = W_tot(n) * (x(n) - x(n-1));
    
        end
        if(x_rest(n) <= 0 && a(n) >= 0) % v(n) <= 0 || 
            % Wenn Destination erreicht ist soll loop verlassen werden.
            break
        end
    end
    x = x(1:n); a = a(1:n); v = v(1:n); x_break = x_break(1:n); F_break = F_break(1:n); x_rest = x_rest(1:n); maxSpeed = maxSpeed(1:n); W_curve = W_curve(1:n);
    F_A = F_A(1:n); F_wheel = F_wheel(1:n); s = s(1:n); W_a = W_a(1:n); W_run = W_run(1:n); W_st = W_st(1:n); W_tot = W_tot(1:n); P_r = P_r(1:n); E = E(1:n); A = A(1:n);
    x_break_2 = x_break_2(1:n);x_break_3 = x_break_3(1:n); v_break_4 = v_break_4(1:n);

    % [x, a, v, x_break, x_rest, s, F_break, F_A, W_run, W_st, W_a, W_tot, mu, F_wheel, P_r, E, A]
    
    Str_sim = array2table([x, x_break, x_rest, s, maxSpeed, x_break_2, x_break_3, v_break_4], 'VariableNames', {'Strecke', 'Bremsstrecke', 'Reststrecke','Steigung', 'MaximalGeschw.', 'Bremsstr2', 'Bremsstr3', 'Bremsgeschw.'}); 

    Fz_sim = array2table([a, v, F_break, F_A, W_run, W_curve, W_st, W_a, W_tot, F_wheel, P_r, E, A], 'VariableNames', {'Beschleunigung', 'Geschwindigkeit', 'Bremskraft', 'Zugkraft', 'Laufwiderstand', 'Kurvenwiderstand', 'Steigungswiderstand', 'Beschleunigungswiderstand', ...
        'Gesamtwiderstand', 'Adhäsionskraft', 'Leistung', 'Energie', 'Traktionsarbeit'});
    CompData = [Str_sim Fz_sim]; % Gesamte simulierte Daten werden ausgelesen

end