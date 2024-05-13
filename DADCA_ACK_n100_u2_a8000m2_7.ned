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

network DADCA_ACK_n100_u2_a8000m2_7
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
        @display("i=misc/sensor;p=455.37,7784.52");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1185.46,7934.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1964.64,7816.72");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2381.45,7999.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2728.15,7945.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3409.09,7754.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4813.83,7999.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5242.89,7874.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5652.95,7833.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=22.61,7653.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=729.59,7508.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1172.81,7556.79");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1590.17,7749.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2542.29,7526.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3815.20,7697.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4212.81,7741.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4679.64,7578.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5953.14,7577.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=313.97,7289.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1506.06,7322.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2105.18,7462.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2924.14,7442.69");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3491.71,7374.91");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3836.37,7313.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4388.63,7364.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5208.24,7314.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1073.19,7161.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1763.30,7066.02");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4605.18,7017.33");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4962.40,7047.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=540.47,6799.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2205.62,6928.55");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2602.97,6954.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3056.91,6902.05");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3436.31,6905.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5681.28,6990.06");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=83.28,6613.92");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=897.68,6603.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3971.09,6731.76");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4278.52,6545.97");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4727.62,6582.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5312.44,6517.58");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5803.47,6653.89");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2321.83,6374.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2884.32,6504.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3267.98,6412.40");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=188.21,6190.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=614.08,6132.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1761.77,6076.13");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3851.45,6212.94");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4278.65,6142.94");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4611.60,6031.18");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5305.96,6074.50");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5720.97,6186.80");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1323.31,5876.99");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2183.06,5945.95");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2728.37,6018.73");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3321.80,5940.35");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3929.43,5868.28");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4954.36,5928.07");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=291.12,5682.64");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=719.22,5729.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1718.42,5586.26");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2651.52,5673.30");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3041.25,5658.30");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4346.82,5601.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4741.88,5538.14");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5450.11,5647.06");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=772.65,5382.34");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1177.37,5477.95");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2232.59,5368.62");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3627.22,5451.20");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5091.52,5460.72");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5624.33,5336.24");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=23.58,5230.37");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=414.96,5268.56");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1690.78,5058.29");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2690.55,5177.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3307.49,5267.07");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4396.88,5202.50");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4862.59,5178.84");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3.87,4843.26");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=492.97,4844.68");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=988.39,4850.24");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2365.78,4808.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3423.79,4812.41");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3894.31,4830.85");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4678.56,4807.83");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5285.45,5015.61");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5931.42,4816.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1839.37,4672.50");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2947.60,4696.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=346.60,4482.71");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=864.66,4496.24");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1240.26,4292.61");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1985.53,4339.31");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3800.05,4441.34");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4440.59,4408.88");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4875.61,4486.41");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5476.97,4502.50");
    }

}

