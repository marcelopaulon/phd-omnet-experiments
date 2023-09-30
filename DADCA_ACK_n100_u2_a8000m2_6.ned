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

network DADCA_ACK_n100_u2_a8000m2_6
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
        @display("i=misc/sensor;p=1602.37,7823.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2669.86,7810.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3805.32,7784.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=338.07,7638.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1818.67,7535.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2162.20,7728.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3121.88,7680.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3507.88,7576.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5525.35,7748.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5951.51,7524.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=530.83,7282.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=930.87,7416.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1396.85,7276.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5161.99,7462.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5607.45,7280.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=169.49,7171.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2204.56,7167.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2785.76,7163.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3270.69,7058.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3949.15,7186.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4959.68,7083.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1167.04,6840.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1777.19,6890.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2973.69,6789.85");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3603.90,6945.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4510.14,6968.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5534.14,6921.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=169.21,6603.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=496.66,6736.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2060.77,6545.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2599.12,6667.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3313.41,6645.89");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3677.87,6525.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4050.63,6682.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=731.81,6402.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1685.28,6280.81");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4587.40,6347.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4976.91,6451.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=0.98,6184.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2149.96,6067.82");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2851.45,6203.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=568.96,5927.06");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1125.46,5984.38");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2593.46,5889.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3208.83,5993.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3685.12,5870.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4120.78,5989.93");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4994.20,5936.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5585.29,5977.06");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=158.35,5682.77");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1215.50,5604.20");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1671.48,5578.53");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3007.06,5645.56");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4119.26,5545.06");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5425.17,5599.65");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=460.17,5462.08");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2070.25,5379.54");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3535.85,5465.88");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4434.52,5384.97");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4842.74,5349.37");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5833.29,5433.75");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=162.14,5042.71");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=772.30,5259.82");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1429.95,5116.86");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2629.91,5268.86");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3101.48,5220.13");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4054.02,5179.92");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5172.38,5195.93");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5500.54,5053.05");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=668.49,4824.29");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1111.79,4840.76");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1890.79,4976.00");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2299.42,5022.36");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3727.62,4956.11");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4382.89,4994.76");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5063.09,4792.82");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1479.82,4730.41");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2537.27,4730.14");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3022.64,4762.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5838.11,4775.34");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=450.27,4308.68");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=850.60,4500.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1214.17,4475.81");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1938.60,4333.01");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2317.12,4363.01");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2771.61,4383.23");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3344.12,4476.01");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4077.13,4371.30");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4760.89,4465.46");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5456.13,4484.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1001.43,4172.66");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3684.35,4221.44");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4399.99,4212.58");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4870.54,4092.02");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5823.38,4184.05");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=328.57,3920.65");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=712.14,3800.21");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1634.54,3987.63");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2538.44,4011.27");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3231.26,3805.83");
    }

}

