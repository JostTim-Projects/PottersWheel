

//const int period_LUT[1024] = {109, 291, 369, 428, 479, 523, 563, 600, 634, 666, 697, 726, 754, 780, 806, 831, 854, 878, 900, 922, 944, 964, 985, 1005, 1024, 1043, 1062, 1081, 1099, 1116, 1134, 1151, 1168, 1185, 1201, 1217, 1233, 1249, 1265, 1280, 1295, 1310, 1325, 1340, 1354, 1369, 1383, 1397, 1411, 1424, 1438, 1452, 1465, 1478, 1491, 1504, 1517, 1530, 1543, 1555, 1568, 1580, 1593, 1605, 1617, 1629, 1641, 1653, 1665, 1676, 1688, 1699, 1711, 1722, 1734, 1745, 1756, 1767, 1778, 1789, 1800, 1811, 1822, 1833, 1843, 1854, 1864, 1875, 1885, 1896, 1906, 1916, 1927, 1937, 1947, 1957, 1967, 1977, 1987, 1997, 2007, 2017, 2027, 2036, 2046, 2056, 2065, 2075, 2084, 2094, 2103, 2113, 2122, 2132, 2141, 2150, 2159, 2169, 2178, 2187, 2196, 2205, 2214, 2223, 2232, 2241, 2250, 2259, 2268, 2277, 2285, 2294, 2303, 2312, 2320, 2329, 2338, 2346, 2355, 2363, 2372, 2380, 2389, 2397, 2406, 2414, 2422, 2431, 2439, 2447, 2456, 2464, 2472, 2480, 2489, 2497, 2505, 2513, 2521, 2529, 2537, 2545, 2553, 2561, 2569, 2577, 2585, 2593, 2601, 2609, 2617, 2625, 2632, 2640, 2648, 2656, 2664, 2671, 2679, 2687, 2694, 2702, 2710, 2717, 2725, 2733, 2740, 2748, 2755, 2763, 2770, 2778, 2785, 2793, 2800, 2808, 2815, 2823, 2830, 2838, 2845, 2852, 2860, 2867, 2874, 2882, 2889, 2896, 2903, 2911, 2918, 2925, 2932, 2940, 2947, 2954, 2961, 2968, 2975, 2983, 2990, 2997, 3004, 3011, 3018, 3025, 3032, 3039, 3046, 3053, 3060, 3067, 3074, 3081, 3088, 3095, 3102, 3109, 3116, 3123, 3130, 3137, 3144, 3151, 3157, 3164, 3171, 3178, 3185, 3192, 3198, 3205, 3212, 3219, 3226, 3232, 3239, 3246, 3253, 3259, 3266, 3273, 3279, 3286, 3293, 3300, 3306, 3313, 3320, 3326, 3333, 3339, 3346, 3353, 3359, 3366, 3372, 3379, 3386, 3392, 3399, 3405, 3412, 3418, 3425, 3432, 3438, 3445, 3451, 3458, 3464, 3471, 3477, 3483, 3490, 3496, 3503, 3509, 3516, 3522, 3529, 3535, 3541, 3548, 3554, 3561, 3567, 3573, 3580, 3586, 3593, 3599, 3605, 3612, 3618, 3624, 3631, 3637, 3643, 3650, 3656, 3662, 3669, 3675, 3681, 3687, 3694, 3700, 3706, 3712, 3719, 3725, 3731, 3737, 3744, 3750, 3756, 3762, 3769, 3775, 3781, 3787, 3793, 3800, 3806, 3812, 3818, 3824, 3831, 3837, 3843, 3849, 3855, 3861, 3868, 3874, 3880, 3886, 3892, 3898, 3904, 3910, 3917, 3923, 3929, 3935, 3941, 3947, 3953, 3959, 3965, 3972, 3978, 3984, 3990, 3996, 4002, 4008, 4014, 4020, 4026, 4032, 4038, 4044, 4050, 4056, 4062, 4068, 4074, 4080, 4086, 4093, 4099, 4105, 4111, 4117, 4123, 4129, 4135, 4141, 4147, 4153, 4159, 4165, 4171, 4177, 4182, 4188, 4194, 4200, 4206, 4212, 4218, 4224, 4230, 4236, 4242, 4248, 4254, 4260, 4266, 4272, 4278, 4284, 4290, 4296, 4302, 4308, 4313, 4319, 4325, 4331, 4337, 4343, 4349, 4355, 4361, 4367, 4373, 4379, 4384, 4390, 4396, 4402, 4408, 4414, 4420, 4426, 4432, 4438, 4443, 4449, 4455, 4461, 4467, 4473, 4479, 4485, 4491, 4496, 4502, 4508, 4514, 4520, 4526, 4532, 4538, 4543, 4549, 4555, 4561, 4567, 4573, 4579, 4585, 4590, 4596, 4602, 4608, 4614, 4620, 4626, 4631, 4637, 4643, 4649, 4655, 4661, 4667, 4672, 4678, 4684, 4690, 4696, 4702, 4708, 4713, 4719, 4725, 4731, 4737, 4743, 4749, 4754, 4760, 4766, 4772, 4778, 4784, 4789, 4795, 4801, 4807, 4813, 4819, 4825, 4830, 4836, 4842, 4848, 4854, 4860, 4866, 4871, 4877, 4883, 4889, 4895, 4901, 4906, 4912, 4918, 4924, 4930, 4936, 4942, 4947, 4953, 4959, 4965, 4971, 4977, 4983, 4988, 4994, 5000, 5006, 5012, 5018, 5024, 5030, 5035, 5041, 5047, 5053, 5059, 5065, 5071, 5076, 5082, 5088, 5094, 5100, 5106, 5112, 5118, 5124, 5129, 5135, 5141, 5147, 5153, 5159, 5165, 5171, 5177, 5182, 5188, 5194, 5200, 5206, 5212, 5218, 5224, 5230, 5236, 5241, 5247, 5253, 5259, 5265, 5271, 5277, 5283, 5289, 5295, 5301, 5307, 5313, 5318, 5324, 5330, 5336, 5342, 5348, 5354, 5360, 5366, 5372, 5378, 5384, 5390, 5396, 5402, 5408, 5414, 5420, 5426, 5432, 5438, 5444, 5450, 5456, 5462, 5468, 5474, 5480, 5486, 5492, 5498, 5504, 5510, 5516, 5522, 5528, 5534, 5540, 5546, 5552, 5558, 5564, 5570, 5576, 5582, 5588, 5594, 5600, 5606, 5612, 5618, 5625, 5631, 5637, 5643, 5649, 5655, 5661, 5667, 5673, 5679, 5685, 5692, 5698, 5704, 5710, 5716, 5722, 5728, 5734, 5741, 5747, 5753, 5759, 5765, 5771, 5778, 5784, 5790, 5796, 5802, 5809, 5815, 5821, 5827, 5833, 5840, 5846, 5852, 5858, 5864, 5871, 5877, 5883, 5889, 5896, 5902, 5908, 5915, 5921, 5927, 5933, 5940, 5946, 5952, 5959, 5965, 5971, 5978, 5984, 5990, 5997, 6003, 6009, 6016, 6022, 6028, 6035, 6041, 6048, 6054, 6060, 6067, 6073, 6080, 6086, 6092, 6099, 6105, 6112, 6118, 6125, 6131, 6138, 6144, 6151, 6157, 6164, 6170, 6177, 6183, 6190, 6196, 6203, 6209, 6216, 6223, 6229, 6236, 6242, 6249, 6255, 6262, 6269, 6275, 6282, 6289, 6295, 6302, 6309, 6315, 6322, 6329, 6335, 6342, 6349, 6356, 6362, 6369, 6376, 6383, 6389, 6396, 6403, 6410, 6417, 6423, 6430, 6437, 6444, 6451, 6458, 6464, 6471, 6478, 6485, 6492, 6499, 6506, 6513, 6520, 6527, 6534, 6541, 6548, 6555, 6562, 6569, 6576, 6583, 6590, 6597, 6604, 6611, 6618, 6626, 6633, 6640, 6647, 6654, 6661, 6669, 6676, 6683, 6690, 6697, 6705, 6712, 6719, 6727, 6734, 6741, 6749, 6756, 6763, 6771, 6778, 6785, 6793, 6800, 6808, 6815, 6823, 6830, 6838, 6845, 6853, 6860, 6868, 6876, 6883, 6891, 6898, 6906, 6914, 6921, 6929, 6937, 6945, 6952, 6960, 6968, 6976, 6984, 6991, 6999, 7007, 7015, 7023, 7031, 7039, 7047, 7055, 7063, 7071, 7079, 7087, 7095, 7103, 7111, 7120, 7128, 7136, 7144, 7153, 7161, 7169, 7177, 7186, 7194, 7203, 7211, 7219, 7228, 7236, 7245, 7254, 7262, 7271, 7279, 7288, 7297, 7305, 7314, 7323, 7332, 7340, 7349, 7358, 7367, 7376, 7385, 7394, 7403, 7412, 7421, 7430, 7440, 7449, 7458, 7467, 7477, 7486, 7495, 7505, 7514, 7524, 7533, 7543, 7552, 7562, 7572, 7582, 7591, 7601, 7611, 7621, 7631, 7641, 7651, 7661, 7671, 7681, 7692, 7702, 7712, 7723, 7733, 7744, 7754, 7765, 7776, 7786, 7797, 7808, 7819, 7830, 7841, 7852, 7863, 7875, 7886, 7897, 7909, 7920, 7932, 7944, 7955, 7967, 7979, 7991, 8003, 8016, 8028, 8040, 8053, 8065, 8078, 8091, 8104, 8117, 8130, 8143, 8157, 8170, 8184, 8197, 8211, 8225, 8240, 8254, 8268, 8283, 8298, 8313, 8328, 8343, 8359, 8375, 8391, 8407, 8423, 8440, 8457, 8474, 8492, 8509, 8528, 8546, 8565, 8584, 8603, 8623, 8644, 8665, 8686, 8708, 8731, 8754, 8778, 8802, 8828, 8855, 8882, 8911, 8942, 8974, 9009, 9045, 9085, 9130, 9180, 9239, 9317, 9500};
