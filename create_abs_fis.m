%% create_abs_fis.m - Builds the Mamdani FIS for ABS Controller
% Reference: Geleta, Gopal & Lemi (2024), Section 2.4, Table 3 & Fig. 5
% Saves the fuzzy inference system to 'abs_fuzzy.fis'

function fis = create_abs_fis(output_filename)
    if nargin < 1 || isempty(output_filename)
        output_filename = 'abs_fuzzy.fis';
    end

    % Check if Fuzzy Logic Toolbox is available
    if ~exist('mamfis', 'file') && ~exist('newfis', 'file')
        error('Fuzzy Logic Toolbox is required to create the FIS.');
    end

    % Use modern mamfis API (R2018b+)
    if exist('mamfis', 'file')
        fis = mamfis('Name', 'abs_fuzzy', ...
                     'AndMethod', 'min', ...
                     'OrMethod', 'max', ...
                     'ImplicationMethod', 'min', ...
                     'AggregationMethod', 'max', ...
                     'DefuzzificationMethod', 'centroid');

        % Input 1: Error = lambda_actual - 0.2
        mf_names = {'NB', 'NM', 'NS', 'ZE', 'PS', 'PM', 'PB'};
        in_centers = [-1.0, -2/3, -1/3, 0.0, 1/3, 2/3, 1.0];
        in_sigma = (1/3) / sqrt(2 * log(2)); % ~ 0.1415 for 0.5 crossover

        fis = addInput(fis, [-1 1], 'Name', 'Error');
        for i = 1:7
            fis = addMF(fis, 'Error', 'gaussmf', [in_sigma, in_centers(i)], 'Name', mf_names{i});
        end

        % Input 2: Error rate = d(Error)/dt
        fis = addInput(fis, [-1 1], 'Name', 'Error rate');
        for i = 1:7
            fis = addMF(fis, 'Error rate', 'gaussmf', [in_sigma, in_centers(i)], 'Name', mf_names{i});
        end

        % Output: Change in pressure (dP)
        % Output sets from Table 3: BPD (HPD), MPD, SPD, PH, SPI, MPI, BPI
        % Scaled to [-500, 500] matching Fig. 5 surface characteristics
        out_names = {'BPD', 'MPD', 'SPD', 'PH', 'SPI', 'MPI', 'BPI'};
        out_scale = 500;
        out_centers = [-1.0, -2/3, -1/3, 0.0, 1/3, 2/3, 1.0] * out_scale;
        out_sigma = in_sigma * out_scale; % ~ 70.77

        fis = addOutput(fis, [-out_scale, out_scale], 'Name', 'Change in pressure');
        for i = 1:7
            fis = addMF(fis, 'Change in pressure', 'gaussmf', [out_sigma, out_centers(i)], 'Name', out_names{i});
        end

        % 19 Rules verbatim from Table 3
        % Input MF indices: 1:NB, 2:NM, 3:NS, 4:ZE, 5:PS, 6:PM, 7:PB
        % Output MF indices: 1:BPD(HPD), 2:MPD, 3:SPD, 4:PH, 5:SPI, 6:MPI, 7:BPI
        % Format: [In1, In2, Out, Weight, Connection(1=AND)]
        rules = [
            4 1 7 1 1;  % 1:  ZE, NB -> BPI
            1 4 7 1 1;  % 2:  NB, ZE -> BPI
            1 5 6 1 1;  % 3:  NB, PS -> MPI
            5 1 6 1 1;  % 4:  PS, NB -> MPI
            4 2 6 1 1;  % 5:  ZE, NM -> MPI
            2 4 6 1 1;  % 6:  NM, ZE -> MPI
            4 3 5 1 1;  % 7:  ZE, NS -> SPI
            3 4 5 1 1;  % 8:  NS, ZE -> SPI
            5 3 4 1 1;  % 9:  PS, NS -> PH
            3 5 4 1 1;  % 10: NS, PS -> PH
            7 3 2 1 1;  % 11: PB, NS -> MPD
            3 7 2 1 1;  % 12: NS, PB -> MPD
            4 4 4 1 1;  % 13: ZE, ZE -> PH
            5 4 3 1 1;  % 14: PS, ZE -> SPD
            4 5 3 1 1;  % 15: ZE, PS -> SPD
            6 4 2 1 1;  % 16: PM, ZE -> MPD
            4 6 2 1 1;  % 17: ZE, PM -> MPD
            7 4 1 1 1;  % 18: PB, ZE -> HPD (BPD)
            4 7 1 1 1;  % 19: ZE, PB -> HPD (BPD)
        ];

        fis = addRule(fis, rules);
        writeFIS(fis, output_filename);
        fprintf('Successfully created and saved %s\n', output_filename);
    else
        % Fallback for legacy MATLAB (newfis)
        fis = newfis('abs_fuzzy', 'mamdani', 'min', 'max', 'min', 'max', 'centroid');
        mf_names = {'NB', 'NM', 'NS', 'ZE', 'PS', 'PM', 'PB'};
        in_centers = [-1.0, -2/3, -1/3, 0.0, 1/3, 2/3, 1.0];
        in_sigma = (1/3) / sqrt(2 * log(2));

        fis = addvar(fis, 'input', 'Error', [-1 1]);
        for i = 1:7
            fis = addmf(fis, 'input', 1, mf_names{i}, 'gaussmf', [in_sigma, in_centers(i)]);
        end

        fis = addvar(fis, 'input', 'Error rate', [-1 1]);
        for i = 1:7
            fis = addmf(fis, 'input', 2, mf_names{i}, 'gaussmf', [in_sigma, in_centers(i)]);
        end

        out_names = {'BPD', 'MPD', 'SPD', 'PH', 'SPI', 'MPI', 'BPI'};
        out_scale = 500;
        out_centers = [-1.0, -2/3, -1/3, 0.0, 1/3, 2/3, 1.0] * out_scale;
        out_sigma = in_sigma * out_scale;

        fis = addvar(fis, 'output', 'Change in pressure', [-out_scale, out_scale]);
        for i = 1:7
            fis = addmf(fis, 'output', 1, out_names{i}, 'gaussmf', [out_sigma, out_centers(i)]);
        end

        rules = [
            4 1 7 1 1; 1 4 7 1 1; 1 5 6 1 1; 5 1 6 1 1; 4 2 6 1 1;
            2 4 6 1 1; 4 3 5 1 1; 3 4 5 1 1; 5 3 4 1 1; 3 5 4 1 1;
            7 3 2 1 1; 3 7 2 1 1; 4 4 4 1 1; 5 4 3 1 1; 4 5 3 1 1;
            6 4 2 1 1; 4 6 2 1 1; 7 4 1 1 1; 4 7 1 1 1
        ];
        fis = addrule(fis, rules);
        writefis(fis, output_filename);
        fprintf('Successfully created and saved %s\n', output_filename);
    end
end
