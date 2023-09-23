package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_u16_a8000m2_6
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=420.23,7912.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=964.76,7950.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2714.58,7814.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4385.66,7800.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4916.45,7965.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5518.83,7917.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2130.81,7714.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3376.62,7728.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3839.48,7613.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5221.64,7704.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=552.39,7441.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1091.88,7384.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1546.35,7408.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2368.51,7392.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2847.51,7458.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4398.81,7428.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5583.97,7361.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=66.75,7179.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=746.00,7038.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1277.98,7072.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2231.02,7051.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3350.21,7097.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3740.03,7257.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4093.78,7098.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5009.33,7165.59");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1828.65,6971.42");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2684.42,6846.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4500.79,6985.86");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5251.00,6875.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=521.56,6633.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1025.17,6711.94");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1378.92,6596.76");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2382.56,6597.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3191.29,6652.60");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3558.19,6605.02");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5773.11,6654.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=310.01,6309.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1073.60,6310.78");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2819.51,6500.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3889.33,6452.83");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4381.88,6431.55");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4876.57,6448.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5373.50,6473.42");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1949.74,6122.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2894.57,6097.10");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3299.27,6167.39");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5082.05,6063.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5496.21,6070.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5833.43,6214.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=248.90,5858.93");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=900.08,5948.30");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1694.56,5823.68");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2358.59,5899.12");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3198.49,5821.21");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3786.11,5986.50");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4133.69,5937.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4693.36,5896.39");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1306.92,5763.73");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2724.05,5529.37");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3766.82,5535.15");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=484.17,5482.85");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=909.90,5464.55");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2116.40,5503.27");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3143.69,5397.62");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4897.96,5429.62");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5402.96,5463.83");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1124.92,5163.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1542.69,5248.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2926.81,5068.95");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3395.85,5068.30");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4219.66,5255.17");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4607.04,5074.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=556.29,5019.76");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1925.69,4952.50");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2334.91,4965.09");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3769.54,5026.96");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4910.17,4794.21");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5224.53,4972.71");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5813.39,4937.87");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=894.67,4558.88");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1320.47,4754.23");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2594.73,4695.13");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3152.85,4706.35");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4202.43,4543.72");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4578.42,4537.96");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5243.00,4619.35");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=245.14,4492.16");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1807.30,4526.13");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2118.28,4356.47");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2940.73,4366.39");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3523.84,4387.81");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3891.22,4345.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5616.77,4441.39");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=113.37,4085.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1385.44,4287.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4759.30,4211.68");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5828.75,4147.85");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=819.10,3970.43");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1472.21,3897.71");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2072.25,3890.78");
    }

}

