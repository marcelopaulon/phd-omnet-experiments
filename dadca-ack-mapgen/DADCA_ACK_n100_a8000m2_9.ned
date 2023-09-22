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

network DADCA_ACK_n100_a8000m2_9
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
        @display("i=misc/sensor;p=560.65,7974.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=736.41,7998.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=119.65,7955.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=239.03,7919.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=499.53,7951.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=655.44,7937.51");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=751.96,7920.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=828.10,7925.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=41.61,7912.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=132.61,7887.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=333.71,7900.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=586.43,7880.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=701.48,7885.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=935.41,7899.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=29.76,7828.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=193.47,7851.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=420.99,7860.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=508.12,7865.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=813.90,7857.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=898.61,7849.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=993.05,7858.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=166.48,7785.86");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=275.86,7810.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=342.89,7801.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=489.13,7799.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=666.84,7820.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=766.15,7799.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=852.88,7797.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=60.43,7768.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=579.47,7771.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=700.96,7751.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=823.18,7741.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=958.87,7762.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=19.97,7706.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=129.31,7722.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=224.29,7704.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=359.27,7706.14");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=474.50,7714.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=634.92,7710.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=884.72,7710.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=422.99,7669.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=535.32,7692.42");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=744.93,7656.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=87.67,7625.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=176.55,7652.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=271.01,7643.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=464.24,7621.30");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=961.15,7649.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=336.70,7571.24");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=525.95,7606.39");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=618.46,7592.23");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=722.90,7578.93");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=811.77,7585.32");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=881.21,7599.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=990.66,7578.16");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=68.78,7548.15");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=196.49,7554.70");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=398.24,7535.41");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=480.52,7561.92");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=545.93,7531.75");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=620.92,7529.04");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=227.47,7484.67");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=300.61,7489.49");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=744.39,7512.95");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=839.71,7489.76");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=941.96,7502.94");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=49.43,7459.36");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=420.76,7467.23");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=485.97,7468.27");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=551.41,7467.90");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=618.63,7444.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=698.38,7447.65");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=230.90,7397.19");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=763.59,7437.31");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=878.50,7405.69");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=941.00,7400.56");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=74.54,7395.05");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=142.56,7382.72");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=447.54,7384.16");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=518.44,7358.57");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=612.14,7378.48");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=681.32,7372.52");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=26.31,7343.39");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=220.90,7333.42");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=292.70,7350.67");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=361.95,7345.76");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=476.77,7313.73");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=766.35,7331.12");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=829.44,7338.12");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=920.54,7326.47");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=109.48,7271.52");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=284.91,7276.86");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=377.47,7272.44");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=561.59,7307.76");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=661.69,7273.30");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=17.57,7223.68");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=183.69,7253.28");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=602.29,7259.07");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=730.70,7253.96");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=835.47,7249.98");
    }

}

