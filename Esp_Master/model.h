#pragma once
#include <cstdarg>
namespace Eloquent {
    namespace ML {
        namespace Port {
            class RandomForest {
                public:
                    /**
                    * Predict class for features vector
                    */
                    int predict(float *x) {
                        uint8_t votes[3] = { 0 };
                        // tree #1
                        if (x[16] <= -6.069999933242798) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[16] <= 2.4399999380111694) {
                                if (x[67] <= -4.070000052452087) {
                                    votes[2] += 1;
                                }

                                else {
                                    if (x[4] <= 4.205000162124634) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[2] += 1;
                                    }
                                }
                            }

                            else {
                                if (x[33] <= -0.6099999994039536) {
                                    votes[1] += 1;
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }
                        }

                        // tree #2
                        if (x[19] <= -6.784999847412109) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[4] <= 4.205000162124634) {
                                if (x[7] <= -4.5299999713897705) {
                                    if (x[64] <= -1.9000000953674316) {
                                        votes[2] += 1;
                                    }

                                    else {
                                        votes[0] += 1;
                                    }
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        // tree #3
                        if (x[1] <= -5.545000076293945) {
                            if (x[37] <= 2.190000057220459) {
                                votes[0] += 1;
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        else {
                            if (x[10] <= 3.8300000429153442) {
                                if (x[31] <= -2.975000023841858) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                if (x[48] <= -1.1850000023841858) {
                                    votes[1] += 1;
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }
                        }

                        // tree #4
                        if (x[7] <= -6.924999952316284) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[37] <= 1.3949999809265137) {
                                if (x[63] <= 0.22999999672174454) {
                                    votes[1] += 1;
                                }

                                else {
                                    if (x[10] <= 3.8300000429153442) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[2] += 1;
                                    }
                                }
                            }

                            else {
                                if (x[25] <= 3.1100000143051147) {
                                    votes[1] += 1;
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }
                        }

                        // tree #5
                        if (x[40] <= -2.690000057220459) {
                            if (x[25] <= -5.555000066757202) {
                                votes[0] += 1;
                            }

                            else {
                                if (x[51] <= 2.3549999594688416) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }
                        }

                        else {
                            if (x[13] <= 4.795000076293945) {
                                if (x[31] <= -3.9000000953674316) {
                                    votes[0] += 1;
                                }

                                else {
                                    if (x[64] <= -3.240000069141388) {
                                        votes[2] += 1;
                                    }

                                    else {
                                        if (x[7] <= 4.064999938011169) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            if (x[86] <= -0.4449999928474426) {
                                                votes[1] += 1;
                                            }

                                            else {
                                                votes[2] += 1;
                                            }
                                        }
                                    }
                                }
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        // tree #6
                        if (x[28] <= -5.230000019073486) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[1] <= 3.315000057220459) {
                                votes[1] += 1;
                            }

                            else {
                                if (x[45] <= -0.6500000059604645) {
                                    if (x[37] <= 2.524999976158142) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[2] += 1;
                                    }
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }
                        }

                        // tree #7
                        if (x[13] <= 2.9000000953674316) {
                            if (x[28] <= -5.230000019073486) {
                                votes[0] += 1;
                            }

                            else {
                                if (x[23] <= -5.5299999713897705) {
                                    votes[2] += 1;
                                }

                                else {
                                    if (x[58] <= -0.7750000059604645) {
                                        if (x[70] <= 1.020000010728836) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[2] += 1;
                                        }
                                    }

                                    else {
                                        votes[1] += 1;
                                    }
                                }
                            }
                        }

                        else {
                            if (x[76] <= 2.6350000500679016) {
                                votes[2] += 1;
                            }

                            else {
                                votes[1] += 1;
                            }
                        }

                        // tree #8
                        if (x[28] <= -5.509999990463257) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[25] <= 3.109999895095825) {
                                if (x[68] <= -0.38500000536441803) {
                                    votes[1] += 1;
                                }

                                else {
                                    if (x[1] <= 3.3300000429153442) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[2] += 1;
                                    }
                                }
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        // tree #9
                        if (x[19] <= -4.710000038146973) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[7] <= 4.064999938011169) {
                                if (x[40] <= 4.145000100135803) {
                                    if (x[89] <= 0.36500000208616257) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        if (x[88] <= 0.22500000149011612) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[2] += 1;
                                        }
                                    }
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        // tree #10
                        if (x[10] <= -5.180000066757202) {
                            if (x[68] <= -1.5550000071525574) {
                                votes[1] += 1;
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        else {
                            if (x[79] <= -0.09999999776482582) {
                                if (x[1] <= 2.7100000381469727) {
                                    if (x[82] <= -2.434999942779541) {
                                        votes[2] += 1;
                                    }

                                    else {
                                        votes[1] += 1;
                                    }
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }

                            else {
                                if (x[1] <= 3.480000138282776) {
                                    votes[1] += 1;
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }
                        }

                        // tree #11
                        if (x[4] <= 3.2100000381469727) {
                            if (x[37] <= -3.8149999380111694) {
                                if (x[66] <= 0.8949999809265137) {
                                    votes[0] += 1;
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                if (x[28] <= -4.309999942779541) {
                                    votes[0] += 1;
                                }

                                else {
                                    if (x[31] <= 5.215000033378601) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[2] += 1;
                                    }
                                }
                            }
                        }

                        else {
                            if (x[42] <= -1.4050000309944153) {
                                votes[1] += 1;
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        // tree #12
                        if (x[75] <= 0.24499999731779099) {
                            if (x[13] <= -3.4100000858306885) {
                                if (x[34] <= -3.305000066757202) {
                                    votes[0] += 1;
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                votes[1] += 1;
                            }
                        }

                        else {
                            if (x[7] <= 2.435000002384186) {
                                if (x[8] <= -1.7049999833106995) {
                                    if (x[10] <= -1.7300000190734863) {
                                        if (x[32] <= -3.190000057220459) {
                                            votes[2] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }

                                    else {
                                        votes[1] += 1;
                                    }
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                if (x[48] <= -0.07500000111758709) {
                                    if (x[32] <= -1.0900000035762787) {
                                        votes[2] += 1;
                                    }

                                    else {
                                        votes[1] += 1;
                                    }
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }
                        }

                        // tree #13
                        if (x[16] <= -3.865000009536743) {
                            if (x[25] <= -4.860000133514404) {
                                votes[0] += 1;
                            }

                            else {
                                votes[1] += 1;
                            }
                        }

                        else {
                            if (x[13] <= 3.2100000381469727) {
                                if (x[50] <= -4.420000076293945) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                if (x[66] <= 3.149999976158142) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }
                        }

                        // tree #14
                        if (x[4] <= -4.980000138282776) {
                            if (x[75] <= 0.8500000238418579) {
                                votes[0] += 1;
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        else {
                            if (x[31] <= 2.4399999380111694) {
                                if (x[86] <= 1.0049999952316284) {
                                    if (x[22] <= 2.1950000524520874) {
                                        if (x[10] <= 4.215000033378601) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[2] += 1;
                                        }
                                    }

                                    else {
                                        if (x[8] <= -2.825000047683716) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[2] += 1;
                                        }
                                    }
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        // tree #15
                        if (x[22] <= -5.480000019073486) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[25] <= 2.2949999570846558) {
                                if (x[4] <= 4.205000162124634) {
                                    votes[1] += 1;
                                }

                                else {
                                    if (x[12] <= 1.8050000071525574) {
                                        votes[2] += 1;
                                    }

                                    else {
                                        votes[1] += 1;
                                    }
                                }
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        // tree #16
                        if (x[27] <= 3.6649999618530273) {
                            if (x[13] <= -2.1550001055002213) {
                                votes[0] += 1;
                            }

                            else {
                                if (x[16] <= 2.5399999618530273) {
                                    votes[1] += 1;
                                }

                                else {
                                    if (x[80] <= -0.9200000166893005) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[2] += 1;
                                    }
                                }
                            }
                        }

                        else {
                            votes[1] += 1;
                        }

                        // tree #17
                        if (x[4] <= 4.819999933242798) {
                            if (x[27] <= 2.1149999499320984) {
                                if (x[16] <= -2.1800000183284283) {
                                    votes[0] += 1;
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                if (x[11] <= -4.329999923706055) {
                                    votes[2] += 1;
                                }

                                else {
                                    if (x[19] <= -7.324999809265137) {
                                        votes[0] += 1;
                                    }

                                    else {
                                        votes[1] += 1;
                                    }
                                }
                            }
                        }

                        else {
                            votes[2] += 1;
                        }

                        // tree #18
                        if (x[22] <= -5.480000019073486) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[75] <= 0.07999999448657036) {
                                votes[1] += 1;
                            }

                            else {
                                if (x[0] <= 3.40500009059906) {
                                    if (x[60] <= 3.7549999952316284) {
                                        votes[2] += 1;
                                    }

                                    else {
                                        votes[1] += 1;
                                    }
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }
                        }

                        // tree #19
                        if (x[7] <= -6.924999952316284) {
                            if (x[29] <= -4.059999942779541) {
                                votes[2] += 1;
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        else {
                            if (x[1] <= 2.7100000381469727) {
                                votes[1] += 1;
                            }

                            else {
                                if (x[16] <= 2.5399999618530273) {
                                    votes[1] += 1;
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }
                        }

                        // tree #20
                        if (x[25] <= -5.150000095367432) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[10] <= 5.894999980926514) {
                                if (x[49] <= -0.9799999892711639) {
                                    if (x[1] <= 5.465000152587891) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[2] += 1;
                                    }
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        // return argmax of votes
                        uint8_t classIdx = 0;
                        float maxVotes = votes[0];

                        for (uint8_t i = 1; i < 3; i++) {
                            if (votes[i] > maxVotes) {
                                classIdx = i;
                                maxVotes = votes[i];
                            }
                        }

                        return classIdx;
                    }

                    /**
                    * Predict readable class name
                    */
                    const char* predictLabel(float *x) {
                        return idxToLabel(predict(x));
                    }

                    /**
                    * Convert class idx to readable name
                    */
                    const char* idxToLabel(uint8_t classIdx) {
                        switch (classIdx) {
                            case 0:
                            return "right";
                            case 1:
                            return "faulty";
                            case 2:
                            return "left";
                            default:
                            return "Houston we have a problem";
                        }
                    }

                protected:
                };
            }
        }
    }