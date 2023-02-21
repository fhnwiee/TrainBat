function [BD] = AuslegungTraktionsBatterie(CR, CompData, const)

% Es wird die neue Leistungskurve mit dem festgesetzten Prozentsatz an zu
% deckender Leistung berechnet
for i = 1:length(CompData.Leistung)
    if (CompData.Leistung(i) > 0)
        P(i,1) = CR/100 * CompData.Leistung(i);
        P(i,2) = CompData.Leistung(i) - P(i,1);
    else
        P(i,1) = 0;
        P(i,2) = CompData.Leistung(i);
    end
end

% Das maximum der Differenz der beiden Kennlinien kann verwendet werden, um
% die maximale Leistung, welche für die Deckung benötigt wird zu bestimmen.
PDiff = CompData.Leistung - P(:,2);
PBatCont = max(PDiff);

% Die Differenz der Energie beider Kennlinien wird verwendet, um die
% Kapazität der Batterie zu bestimmen.
EBatCont = sum(const.dt * P(:,1));

% Es werden automatisch verschiedene Prozentsätze berechnet und danach
% miteinander verglichen.
%CRset = [0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]; % zu berechnende Prozentsätze
CRset = [0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5];

for n = 1:length(CRset)
    for i = 1:length(CompData.Leistung)
        if (CompData.Leistung(i) > 0)
            P(i,n) = CRset(n) * CompData.Leistung(i);
            Psub(i,n) = CompData.Leistung(i) - P(i,n);
        else
            P(i,n) = 0;
            Psub(i,n) = CompData.Leistung(i);
        end
    end
    PDiff(:,n) = CompData.Leistung - Psub(:,n);
    PBatCont(:,n) = max(PDiff(:,n));
    EBatCont(n) = sum(const.dt * P(:,n));
end



% Berechnung der Gesamtfläche:
% Es wird das Matlab Package "Fast and Robust Curve Intersections" von
% Douglas Schwarz benötigt.

for i = 1:length(CompData.Strecke)
    if (CompData.Leistung(i) > 0)
        Etot(i,1) = CompData.Energie(i);
    else
        Etot(i,1) = 0;
    end
end
ETOT = sum(Etot);

inters = max(CompData.Leistung)-10; % the intersection line

EBatPeak = 0;

while (ETOT * CR/100 > EBatPeak)
    EBatPeak = 0;
    inters = inters - 1;
    [xlin, ylin] = intersections(1:length(CompData.Leistung),CompData.Leistung, [1, length(CompData.Leistung)], [inters inters]);
    xlin = round(xlin);

    if(length(xlin) > 2)

        for i = 1:2:length(xlin)

            EBatPeak = EBatPeak + sum(CompData.Leistung(xlin(i):xlin(i+1))-inters)*const.dt;

        end
    else
        EBatPeak = sum(CompData.Leistung(xlin(1):xlin(2))-inters)*const.dt;
    end

    if (ETOT * CR/100 < 0.999 * EBatPeak)
        inters = inters + 1.5;
        EBatPeak = 0;
    end

end

% Die benötigte Batterieleistung kann nun berechnet werden, um die
% berechnete Leistungsspitze zu decken.
PBatPeak = max(CompData.Leistung) - inters;

% Ebenfalls kann die benötigte Energie, welche für die Deckung benötigt
% wird berechnet werden:
EBatPeak;

% Es werden automatisch verschiedene Prozentsätze berechnet und danach
% miteinander verglichen.
%CRset = [0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]; % zu berechnende Prozentsätze
CRset = [0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5];

inters(1:length(CRset)) = max(CompData.Leistung)-10;

EBatPeak(1:length(CRset)) = 0;

for n = 1:length(CRset)
    EBatPeak(n) = 0;

    while (ETOT * CRset(n) > EBatPeak(n))
        EBatPeak(n) = 0;
        inters(n) = inters(n) - 1;
        [xlin, ylin] = intersections(1:length(CompData.Leistung),CompData.Leistung, [1, length(CompData.Leistung)], [inters(n) inters(n)]);
        xlin = round(xlin);

        if(length(xlin) > 2)

            for i = 1:2:length(xlin)

                EBatPeak(n) = EBatPeak(n) + sum(CompData.Leistung(xlin(i):xlin(i+1))-inters(n))*const.dt;

            end
            
        else
            EBatPeak(n) = sum(CompData.Leistung(xlin(1):xlin(2))-inters(n))*const.dt;
        end

        PBatPeak(n) = max(CompData.Leistung) - inters(n);
   
    end
end

for i = 1:length(CRset)
    BD.("CR_" + string(CRset(i)*100) + "_Prozent").EBatCont = EBatCont(i);
    BD.("CR_" + string(CRset(i)*100) + "_Prozent").EBatPeak = EBatPeak(i);
    BD.("CR_" + string(CRset(i)*100) + "_Prozent").ETOT = ETOT;
    BD.("CR_" + string(CRset(i)*100) + "_Prozent").inters = inters(i);
    BD.("CR_" + string(CRset(i)*100) + "_Prozent").PBatCont = PBatCont(i);
    BD.("CR_" + string(CRset(i)*100) + "_Prozent").Psub = Psub(:,i);
    BD.("CR_" + string(CRset(i)*100) + "_Prozent").PDiff = PDiff(:,i);
    BD.("CR_" + string(CRset(i)*100) + "_Prozent").PBatPeak = PBatPeak(i);
end

end