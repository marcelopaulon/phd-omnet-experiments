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

network DADCA_ACK_n100_a8000m2_5
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
        @display("i=misc/sensor;p=79.11,7983.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=162.08,7971.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=297.95,7975.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=470.21,7982.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=597.07,7974.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=740.06,7986.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=931.57,7973.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=65.04,7921.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=446.09,7922.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=647.82,7932.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=727.20,7920.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=789.73,7932.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1.84,7905.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=209.37,7912.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=540.31,7896.79");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=852.01,7875.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=973.12,7883.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=104.91,7869.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=212.42,7845.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=328.20,7866.76");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=448.46,7854.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=513.79,7831.50");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=641.12,7845.13");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=741.59,7857.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=141.31,7810.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=321.71,7803.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=406.63,7784.92");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=697.37,7786.30");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=786.18,7802.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=893.33,7817.27");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=966.75,7820.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=96.88,7748.36");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=241.07,7751.79");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=494.09,7742.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=577.53,7777.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=861.60,7747.07");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=974.03,7749.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=149.15,7710.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=304.08,7738.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=369.20,7718.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=624.58,7714.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=737.42,7704.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=43.88,7679.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=218.52,7661.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=277.23,7680.64");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=427.40,7687.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=560.68,7691.89");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=793.82,7674.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=957.26,7660.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=72.28,7617.28");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=326.43,7629.22");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=472.29,7641.89");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=666.35,7625.98");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=783.64,7613.84");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=169.61,7598.98");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=280.90,7576.07");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=386.74,7609.52");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=573.09,7593.87");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=848.14,7601.43");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=923.56,7609.31");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=67.58,7547.39");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=431.64,7553.55");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=725.28,7564.38");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=961.02,7543.46");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5.67,7511.02");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=169.98,7496.22");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=228.37,7521.83");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=373.76,7511.20");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=521.59,7523.10");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=678.88,7506.79");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=836.44,7490.45");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=899.96,7525.01");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=244.15,7447.25");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=438.04,7455.88");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=562.87,7462.30");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=913.01,7441.70");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=981.50,7468.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=63.49,7424.10");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=347.54,7429.65");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=511.63,7422.22");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=775.38,7420.15");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=25.40,7367.51");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=132.84,7369.05");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=348.67,7360.25");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=597.67,7357.55");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=665.02,7391.87");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=873.04,7380.51");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=80.50,7335.55");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=235.00,7347.62");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=424.67,7350.05");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=755.79,7330.09");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=825.36,7335.08");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=941.94,7325.74");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=7.96,7304.91");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=116.81,7283.38");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=328.58,7275.53");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=397.28,7281.83");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=680.11,7292.90");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=47.43,7242.93");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=457.17,7242.39");
    }

}

