function [Str_sim, Fz_sim, CompData] = Fahrtsimulation11(Str, FZ, const, Fahrtzeit)
% Diese Funktion simuliert eine Strecke einer Zugfahrt schrittweise. In
% jedem Zeitschritt werden die Parameter neu berechnet. Für die
% Berechnungen werden die Strecken- und Fahrzeugparameter, festgelegte
% Konstanten, sowie die vorher mit der Funktion "Optimization.mlx"
% berechneten Grenzwerte benötigt.

% V11
% Änderung damit json und eigenes Format zur Berechnung verwendet werden können.
% Anpassungen der Variablennamen, damit auch die json-Dateien verwendet werden können
% Es werden die Parameter von den Json-Dateien übernommen.
% Es wurde eine Rampe für das Beschleunigen und Bremsen der Fahrzeuge implementiert.
% Das Ziel wird nicht mehr so schnell wie möglich sondern nach Zeitplan
% erreicht.

% Vorgabe der Matizengrösse (bei langen Strecken muss die Grösse angepasst
% werden (mehr als 20000 Berechnungsschritte)
mat = zeros(20000,1);

% Erzeugen von leeren Matrizen um die Daten zu schreiben (ohne bereits
% vorgegebene Felder dauert die Rechenzeit länger)
x = mat;  a = mat; v = mat; s = mat; x_rest = mat; F_brake = mat; F_A = mat; W_run = mat; W_st = mat; W_a = mat; W_curve = mat;
W_tot = mat; mu = mat; F_wheel = mat; P_r = mat; E = mat; A = mat; maxSpeed = mat; r = mat; vBeschl = mat; speedLimit = mat; xBrake = mat;
Bremsstrecke = mat;

% Es wird überprüft, ob der Parameter "M_axle" in den Fahrzeugparametern
% hinterlegt wurde
if(isfield(FZ, 'M_axle'))
    % Berechnung falls Parameter hinterlegt
    G_T = FZ.M_axle * FZ.N_axles * const.g; % (kN) Gewicht, welches durch die angetriebenen Achsen übertragen werden kann.
else
    % Berechnung falls Parameter nicht hinterlegt (Annahme Achslast =
    % 19.5t)
    G_T = 19.5 * 4 * const.g; % (kN) Gewicht, welches durch die angetriebenen Achsen übertragen werden kann.
end

% Überprüfen ob Parameter "mu_brake" hinterlegt wurde. Falls nicht wird ein
% Wert für den Reibwert von Rad/Schiene für das Bremsen angenommen (Bsp.
% 0.15)
if(isfield(FZ, 'mu_brake') == false)
    FZ.mu_brake = 0.15; % (-)
end

% Überprüfen ob Parameter "maxAcceleration" hinterlegt wurde. Ansonsten
% wird ein Wert für die max. Beschleunigung angenommen (Bsp. 0.8 m/s^2)
if(isequal(FZ.maxAcceleration,[]))
    FZ.maxAcceleration = 0.8; % (m/s^2)
end

time = 0;
% loop um die Streckensimulation durchzuführen. Die Simulation wird solange
% wiederhohlt, bis die vorgegebene Fahrtzeit eingehalten wird.
while(time < Fahrtzeit)
    time = 0;
    for n = 1:200000

        % Berechnung von Startwerten zu Beginn der Simulation
        if (n == 1)
            % Startgeschwindigkeit = 0
            v(n) = 0; % (km/h)
            % Startbeschleunigkeit = Beschleunigungsschritt (0.1 m/s^2)
            a(n) = const.accRamp; % (m/s^2)
            % Zurückgelegte Strecke = 0
            x(n) = 0; % (m)
            % Reststrecke = Gesamtstrecke
            x_rest(n) = Str.stations(end) - Str.stations(1) - x(n); % (m)
            % zur Verfügung stehende Zugkraft = max. Zugkraft
            F_A(n) = FZ.maxTractionForce; % (kN)
            % Es soll nicht gebremst werden.
            STOP = false;
        else
            % aktuelle Geschwindigkeit mit Beschleunigung des vorherigen Zeitschrittes über die Zeitdifferenz
            v(n) = a(n-1)*const.dt * 3.6 + v(n-1); % (km/h)

            % zurückgelegte und restliche Strecke berechnen
            x(n) = x(n-1) + const.dt * v(n)/3.6; % (m)
            x_rest(n) = Str.stations(end) - Str.stations(1) - x(n); % (m)

            % Steigung des aktuellen Streckenabschnittes bestimmen
            for st = 1:length(Str.gradients)
                if (st == length(Str.gradients))
                    % im letzten Streckenabschnitt wird die letzte Steigung
                    % benötigt
                    s(n) = Str.gradients(st, 2); % (‰)
                    break
                elseif (Str.gradients(st + 1, 1) >= x(n))
                    % Finden der im aktuellen Streckenbereich hinterlegten
                    % Steigung in Promille
                    s(n) = Str.gradients(st, 2); % (‰)
                    break
                end
            end

            % aktuelle Geschwindigkeitslimite bestimmen (km/h)
            for Limit = 1:length(Str.speedLimits)
                if (Limit == length(Str.speedLimits))
                    % im letzten Streckenabschnitt wird die letzte max.
                    % Geschwindigkeit benötigt.
                    maxSpeed(n) = Str.speedLimits(Limit, 2); % (km/h)
                    break
                elseif (Str.speedLimits(Limit + 1, 1) >= x(n))
                    % Finden der im aktuellen Streckenbereich hinterlegten max.
                    % Geschwindigkeit
                    maxSpeed(n) = Str.speedLimits(Limit, 2); % (km/h)
                    break
                end
                % Abspeichern der aktuellen max. Geschwindigkeit des Zeitschritts.
                speedLimit(n) = maxSpeed(n); % (km/h)
            end
            % maxSpeed(n) = 75; % max. Geschw. kann auch hier manuell
            % hinterlegt werden

            % Aktuelle Zugkraft berechnen
            if (FZ.maxTractionForce * v(n)/3.6 <= FZ.maxTractionPower)
                % Wenn Traktionsleistung kleiner ist als die max.
                % Traktionsleistung entspricht die Zugkraft der max. Zugkraft.
                F_A(n) = FZ.maxTractionForce; % (kN)
            else
                % Berechnung der zur Verfügung stehenden Zugkraft
                F_A(n) = FZ.maxTractionPower/(v(n)/3.6); % (kN)
            end

            % Reibungskoeffizient berechnen
            mu(n) = 7.5/(v(n) + 44) + 0.161; % (-)

            % maximal übertragbare Kraft aufgrund von Adhäsion
            F_wheel(n) = mu(n) * G_T; % (kN)

            % Laufwiderstand berechnen:
            if(isfield(FZ,'N_waggon'))
                % Berechnung falls Parameter "N_waggon" angegeben ist.
                % (Sauthoff Formel)
                W_run(n) = (1.9 + 0.0025 * v(n) + 4.8 * (FZ.N_waggon + 2.7)/(FZ.mass * const.g) * 0.0145 * (v(n) + 15)^2) * FZ.mass * const.g / 1000; % (kN)
            else
                % Berechnung mit Polynomformel
                W_run(n) = (FZ.rollingResistanceR0 + v(n) * FZ.rollingResistanceR1 + (v(n))^2 * FZ.rollingResistanceR2)/1000; % (kN)
            end

            % Kurvenwiderstand berechnen:
            if(isfield(Str,'curvatures'))
                % Berechnung wird nur durchgeführt, wenn das Kurvenprofil in den
                % Streckendaten hinterlegt wurde.
                for c = 1:length(Str.curvatures)
                    if (c == length(Str.curvatures))
                        % Streckenradius der letzten Kurve
                        r(n) = (Str.curvatures(c,2) + Str.curvatures(c,3))/2; % (m)
                        break
                    elseif (Str.curvatures(c + 1, 1) >= x(n))
                        % Auswahl der zum Streckenabschnitt zugehörigen
                        % Kurvenradien
                        r(n) = (Str.curvatures(c,2) + Str.curvatures(c,3))/2; % (m)
                        break
                    end
                end
                if (r(n) < 300)
                    % Kurvenwiderstand bei r < 300 m (Röckl'sche Formel)
                    W_curve(n) = (FZ.mass * const.g * 650/(r(n)-55))/1000; % (kN)
                else
                    % Kurvenwiderstand bei r >= 300 m (Röckl'sche Formel)
                    W_curve(n) = (FZ.mass * const.g * 500/(r(n)-30))/1000; % (kN)
                end
            else
                % Falls keine Kurvenangaben in den Daten hinterlegt sind wird
                % der Kurvenwiderstand auf Null gesetzt.
                W_curve(n) = 0;
            end

            % Steigungswiderstand berechnen:
            W_st(n) = FZ.mass * const.g * s(n)/1000; % (kN)

            % Berechnung ab welcher Geschwindigkeit runterbeschleunigt werden muss um Maximalgeschwindigkeit einzuhalten
            vBeschl(n) = maxSpeed(n);
            for i = 0:const.accRamp:a(n-1)
                vBeschl(n) = vBeschl(n) - i * const.dt * 3.6; % (km/h)
            end

            % Berechnung der aktuellen Bremsstrecke: (m)
            if (STOP == false)
                [Bremsstrecke(n), ~, vBrems] = CalcBremsstreckeV3(v(n), const, FZ, a(n-1));
            end

            % Entscheidung ob gebremst oder beschleunigt werden muss. Der
            % Bremsvorgang wird eingeleitet, sobald die Reststrecke der im
            % vorhinein berechneten Bremsstrecke unterschreitet.
            if (Bremsstrecke(n) >= x_rest(n))
                STOP = true;
            elseif (Bremsstrecke(n) < x_rest(n))
                STOP = false;
            end

            % Bremsen oder Beschleunigen
            % Wenn Bremsstrecke grösser oder gleich ist wie die Reststrecke muss gebremst werden.
            if(STOP == true)
                % Berechnung der Bremskraft des Fahrzeugs: F_brake
                %F_brake(n) = -FZ.mu_brake * FZ.mass * const.g * cos(s(n)/1000); % (kN)
                F_brake(n) = -FZ.mu_brake * G_T * const.g * cos(s(n)/1000); % (kN)

                % Wenn die minimale Geschwindigkeit für das Reduzieren der
                % Beschleunigung erreicht ist:
                if (vBrems >= v(n))
                    % Die Beschleunigung wird schrittweise reduziert
                    a(n) = a(n-1) + const.accRamp; % (m/s^2)
                else
                    % Wenn Geschwindigkeit und die neue Geschwindigkist grösser als 0 ist, darf gebremst werden.
                    if(v(n) > 0 && (FZ.maxDeceleration * const.dt * 3.6 + v(n)) > 0)

                        % Wenn die neu berechnete Verzögerung grösser ist
                        % als die max. Verzögerung:
                        if (F_brake(n)/FZ.mass + const.g * sin(s(n)/1000) < FZ.maxDeceleration)

                            % Beschleunigung des vorherigen Schrittes minus
                            % Beschleunigungsschritt kleiner als die max.
                            % Verzögerung
                            if (a(n-1) - const.accRamp <= FZ.maxDeceleration)
                                % neue Verzögerung entspricht max. Verzögerung
                                a(n) = FZ.maxDeceleration; % (m/s^2)
                            else
                                % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                                a(n) = a(n-1) - const.accRamp; % (m/s^2)
                            end
                        else % neu berechnete Verzögerung ist grösser als max. Verzögerung
                            if (a(n-1) - const.accRamp <= F_brake(n)/FZ.mass + const.g * sin(s(n)/1000))
                                % Wenn Verzögerung aufgrund der Bremskraft kleiner als max. Bremsverzögerung ist.
                                a(n) = F_brake(n)/FZ.mass + const.g * sin(s(n)/1000); % (m/s^2)
                            else
                                % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                                a(n) = a(n-1) - const.accRamp; % (m/s^2)
                            end
                        end
                        % Wenn mit max. Verzögerung eine Geschwindigkeit von
                        % weniger als 0 erreicht werden würde:
                    elseif(FZ.maxDeceleration * (const.dt * 3.6 + v(n)) < 0 && v(n) > 0)
                        % Berechnung der Beschl. wenn mit max. Beschleunigung negative Geschw. erreicht wird und Geschw. > 0 ist.
                        if (a(n-1) - const.accRamp <= -v(n)/(const.dt * 3.6))
                            % neue Berechnung der Verzögerung
                            a(n) = -v(n)/(const.dt * 3.6); % (m/s^2)
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) - const.accRamp; % (m/s^2)
                        end
                    else % Wenn Geschwindigkeit 0 oder kleiner als 0 ist darf nicht mehr gebremst werden.
                        a(n) = 0;
                    end
                end
                % Wenn die Geschw. grösser ist als der Grenzwert vBeschl und die
                % Beschl. grösser als 0 ist muss die Beschleunigung reduziert
                % werden.
            elseif (v(n) >= vBeschl(n) && a(n-1) > 0)
                % Beschleunigung wird langsam reduziert.
                a(n) = a(n-1) - const.accRamp; % (m/s^2)

                % Es darf nicht gebremst werden --> Beschleunigen und konstante
                % Fahrt:
            elseif (STOP == false)

                % Wenn Gesamtwiderstand grösser ist als Zugkraft:
                if(W_tot(n-1) > F_A(n-1))
                    % Beschleunigung muss reduziert werden
                    a(n) = a(n-1) - const.accRamp; % (m/s^2)

                    % Wenn Zugkraft minus Lauf-, Steigungs- und Kurvenwiderstand
                    % grösser als Null ist und die Kraft von Rad auf Schiene
                    % ebenfalls grösser ist wie der Widerstand:
                elseif ((F_A(n) - (W_run(n) + W_st(n) + W_curve(n))) > 0 && F_wheel(n) >= (W_run(n) + W_st(n) + W_curve(n)))

                    % wenn theoretisch mögliche Beschleunigung grösser als max.
                    % Beschleunigung, Geschw. kleiner als Geschw.limit und
                    % Kraftübertragung aufs Rad grösser als Zugkraft:
                    if (((F_A(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)) > FZ.maxAcceleration && v(n) < maxSpeed(n) && F_wheel(n) > F_A(n))
                        % Wenn Beschleunigung grösser als max. Beschleunigung
                        if (a(n-1) >= FZ.maxAcceleration - const.accRamp)
                            % max. Beschleunigung wenn genügend Kraft zur Verfügung steht und Maximalgeschw. nicht erreicht ist
                            a(n) = FZ.maxAcceleration; % (m/s^2)
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) + const.accRamp; % (m/s^2)
                        end
                        % Wenn Geschw. kleiner max. Geschw., Kraft auf Schiene
                        % kleiner als Zugkraft, und mögliche Beschleunigung grösser
                        % als max. Beschleunigung dann:
                    elseif (v(n) < maxSpeed(n) && F_wheel(n) < F_A(n) && ((F_wheel(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)) > FZ.maxAcceleration)
                        % max. Beschleunigung wenn Kraft auf Gleise kleiner als Zugkraft, diese jedoch genügend gross um Max.
                        % Beschleunigung zu erreichen. Geschw. < als max. Geschwindigkeit
                        if (a(n-1) >= FZ.maxAcceleration - const.accRamp)
                            % max. Beschleunigung wenn genügend Kraft zur Verfügung steht und Maximalgeschw. nicht erreicht ist
                            a(n) = FZ.maxAcceleration; % (m/s^2)
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) + const.accRamp; % (m/s^2)
                        end

                        % Gesch. kleiner als max. Geschw., Kraft auf Schiene
                        % kleiner als Zugkraft, mögliche Beschl. kleiner als max.
                        % Beschleunigung:
                    elseif (v(n) < maxSpeed(n) && F_wheel(n) < F_A(n) && ((F_wheel(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)) < FZ.maxAcceleration)
                        % Beschleunigung wird berechnet über max. übertragbare Kraft auf Gleise, wenn diese kleiner
                        % ist als die Zugkraft. Geschw. < max. Geschw.
                        if(a(n-1) + const.accRamp >= ((F_wheel(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)))
                            % Berechnung der neuen Beschleunigung
                            a(n) = ((F_wheel(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)); % (m/s^2)
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) + const.accRamp; % (m/s^2)
                        end

                        % Wenn Geschwindigkeit kleiner als Geschwindigkeitsgrenze
                    elseif (v(n) < maxSpeed(n))
                        % Wenn Beschleunigung begrenzt wird durch zur Verfügung stehende Kraft wird sie berechnet
                        if (a(n-1) + const.accRamp >= ((F_A(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)))
                            a(n) = ((F_A(n) - (W_run(n) + W_st(n) + W_curve(n)))/(FZ.mass * FZ.rho)); % (m/s^2)
                        else
                            % Rampe für das Bremsen wenn noch nicht maximale Bremsbeschleunigung erreicht ist
                            a(n) = a(n-1) + const.accRamp; % (m/s^2)
                        end

                    else
                        % Wenn aktuelle Geschw. > max. Geschw.
                        if(a(n-1) > const.accRamp)
                            % Beschleunigung muss reduziert werden
                            a(n) = a(n-1)-const.accRamp; % (m/s^2)
                        else
                            a(n) = 0;
                        end
                    end

                else
                    if(a(n-1) > const.accRamp)
                        % Beschleunigung muss reduziert werden
                        a(n) = a(n-1)-const.accRamp; % (m/s^2)
                    else
                        a(n) = 0;
                    end
                end
            else
                if(abs(a(n-1)) > const.accRamp)
                    a(n) = a(n-1) + const.accRamp; % (m/s^2)
                else
                    a(n) = 0;
                end
            end

            % Beschleunigungswiderstand berechnen:
            W_a(n) = FZ.mass * a(n) * FZ.rho; % (kN)

            % Gesamtwiderstand berechnen:
            W_tot(n) = W_run(n) + W_st(n) + W_a(n) + W_curve(n); % (kN)

            % Berechnung der Leistung während der Fahrt (negative Leistung
            % beim Bremsen wird Rekuperiert. Bei v < 5 km/h wird nur
            % pneumatische Bremse eingesetzt. Der Grenzwert kann jedoch geänder
            % werden (FZ.minSpeedRegBraking)
            % Wenn Leistung grösser als Null:
            if (W_tot(n) * v(n)/3.6 >= 0)
                % Leistung die gebraucht wird für Traktion
                P_r(n) = ((W_tot(n) * v(n)/3.6) + const.AuxP)/const.nPantograph; % (kW)
                % Wenn Leistung negativ und Geschw. grösser als Grenzwert für
                % regeneratives Bremsen:
            elseif (W_tot(n) * v(n)/3.6 < 0 && v(n) > FZ.minSpeedRegBraking)
                if(abs((W_tot(n) * v(n)/3.6 + const.AuxP) * const.nPantograph) <= FZ.maxRegPower)
                    % Regenerative, negative Leistung
                    P_r(n) = (W_tot(n) * v(n)/3.6 + const.AuxP) * const.nPantograph; % (kW)
                else
                    % Wenn negative Leistung grösser ist als die max.
                    % regenerative Leistung:
                    P_r(n) = -FZ.maxRegPower; % (kW)
                end
            else
                P_r(n) = (0 + const.AuxP) * const.nPantograph; % (kW)
            end

            % Berechnung der während der Fahrt benötigten Energie
            E(n) = P_r(n) * const.dt; % (kJ)

            % Berechnung der geleisteten Traktionsarbeit 
            A(n) = W_tot(n) * (x(n) - x(n-1)); % (kJ)

        end

        % Bedingung für das Beenden der Simulation
        if(x_rest(n) <= 1 && a(n) > -const.accRamp)
            % Wenn Destination erreicht ist soll loop verlassen werden.
            break
        end

    end
    Str.speedLimits(:,2) = Str.speedLimits(:,2) - 0.1;
    time = n*const.dt;
end

% Matrizen werden auf die länge der Simulation gesetzt:
xBrake = Bremsstrecke;
xBrake = xBrake(1:n);
x = x(1:n); a = a(1:n); v = v(1:n); F_brake = F_brake(1:n); x_rest = x_rest(1:n); maxSpeed = maxSpeed(1:n); W_curve = W_curve(1:n);
F_A = F_A(1:n); F_wheel = F_wheel(1:n); s = s(1:n); W_a = W_a(1:n); W_run = W_run(1:n); W_st = W_st(1:n); W_tot = W_tot(1:n); P_r = P_r(1:n); E = E(1:n); A = A(1:n);
speedLimit = speedLimit(1:n); % vBeschl = vBeschl(1:n);


% Streckenparameter werden in Tabelle gespeichert:
Str_sim = array2table([x, xBrake(1:n), x_rest, s, maxSpeed, speedLimit], 'VariableNames', {'Strecke', 'Bremsstrecke', 'Reststrecke','Steigung', 'vLimit', 'Maximalgeschw.'});

% Fahrzeugparameter werden in Tabelle gepeichert:
Fz_sim = array2table([a, v, F_brake, F_A, W_run, W_curve, W_st, W_a, W_tot, F_wheel, P_r, E, A], 'VariableNames', {'Beschleunigung', 'Geschwindigkeit', 'Bremskraft', 'Zugkraft', 'Laufwiderstand', 'Kurvenwiderstand', 'Steigungswiderstand', 'Beschleunigungswiderstand', ...
    'Gesamtwiderstand', 'Adhäsionskraft', 'Leistung', 'Energie', 'Traktionsarbeit'});

% Zusammenfassung aller Parameter in einer Tabelle:
CompData = [Str_sim Fz_sim]; % Gesamte simulierte Daten werden ausgelesen


end