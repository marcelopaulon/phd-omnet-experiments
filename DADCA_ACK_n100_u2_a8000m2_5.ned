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

network DADCA_ACK_n100_u2_a8000m2_5
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
        @display("i=misc/sensor;p=162.76,7990.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1273.13,7850.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1874.36,7947.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2835.15,7977.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3205.53,7982.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4270.00,7991.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=385.37,7635.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2429.12,7547.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3130.85,7509.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3583.56,7564.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4124.26,7567.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4744.82,7623.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5329.62,7646.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=918.93,7361.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1421.50,7427.72");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1791.74,7439.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2830.19,7282.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=368.04,7239.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1535.52,7013.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2399.82,7162.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3339.06,7196.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3709.50,7224.02");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5198.81,7163.82");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5619.85,7112.08");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=322.05,6859.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2878.00,6862.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3232.10,6848.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4271.42,6888.50");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4784.33,6806.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5927.37,6924.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1009.91,6679.19");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1531.22,6579.91");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1938.80,6712.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2280.46,6573.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2693.32,6528.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3648.05,6723.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4018.00,6535.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5463.52,6655.71");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5950.64,6556.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=412.05,6391.27");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3147.22,6309.35");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5158.11,6324.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5604.59,6325.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=68.76,6221.27");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1046.94,6042.09");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1432.49,6224.05");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1863.25,6257.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2195.26,6040.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3809.54,6149.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4483.85,6233.19");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5989.05,6023.68");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=28.99,5843.14");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=699.02,5994.40");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2649.55,6003.72");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4253.00,5882.16");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4835.81,5806.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5412.30,5989.14");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=443.52,5718.81");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1170.47,5599.13");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1595.01,5754.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2214.40,5659.81");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3036.81,5690.32");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3390.07,5541.70");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1770.13,5418.65");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2500.30,5312.82");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3762.29,5480.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4481.66,5305.97");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5214.48,5470.08");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5676.05,5343.03");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=104.20,5159.91");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=836.53,5244.44");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1232.21,5251.34");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3149.44,5162.19");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4209.87,5030.95");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5148.71,5050.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=703.29,4879.76");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1653.81,4997.22");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2172.59,5018.08");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2797.40,4819.32");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3486.45,5007.53");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5644.67,4933.94");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1278.52,4727.58");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2340.96,4681.34");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3095.51,4609.62");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3851.42,4705.49");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4675.36,4591.71");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5160.07,4613.62");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=322.77,4531.63");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=943.66,4328.00");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1490.08,4365.33");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1929.90,4472.43");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4208.67,4414.76");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=112.46,4101.32");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3202.17,4170.26");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4871.39,4094.43");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5710.55,4243.41");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1010.61,3870.76");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1391.42,3852.43");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1745.05,3864.86");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2727.96,3931.45");
    }

}

