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

network DADCA_ACK_n100_u8_a8000m2_5
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
        @display("i=misc/sensor;p=717.27,7797.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1199.23,7758.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2265.80,7882.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2615.72,7788.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2936.79,7973.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4014.47,7981.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4652.90,7928.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5293.55,7954.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=326.98,7679.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1765.83,7650.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3589.42,7643.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4011.12,7629.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5128.92,7590.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=605.63,7402.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1000.83,7371.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1355.26,7318.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2139.23,7319.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2475.41,7465.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2841.58,7495.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3246.89,7470.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4651.47,7482.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5620.22,7450.18");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=83.17,7149.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=442.87,7038.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1777.55,7215.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2495.93,7052.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3698.90,7216.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4890.43,7025.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5289.15,7167.33");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5959.51,7205.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=123.78,6792.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1283.23,6972.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2121.98,6949.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3316.16,6804.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4028.28,6966.82");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5934.74,6828.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=447.07,6600.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=887.41,6727.79");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1818.21,6748.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2177.67,6602.62");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2531.84,6523.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3941.30,6566.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4593.24,6567.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5254.41,6528.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=31.12,6316.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1263.46,6324.01");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2874.90,6431.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3499.25,6442.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4275.09,6378.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1919.54,6230.41");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2570.27,6119.45");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2987.76,6030.82");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4579.82,6057.51");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4984.23,6145.54");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5654.11,6073.23");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=6.24,5774.20");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=486.94,5850.63");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1100.66,5981.10");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1450.27,5844.75");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3647.64,5801.45");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4098.84,5785.37");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1064.09,5613.54");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2088.29,5612.95");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2706.46,5627.82");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4510.97,5683.51");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5298.80,5536.32");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5836.54,5627.05");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=7.50,5378.52");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=444.83,5354.60");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1481.03,5449.31");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3386.00,5354.56");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4846.97,5452.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=994.56,5035.55");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1706.81,5087.86");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2198.89,5121.24");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2996.43,5094.45");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4075.17,5242.33");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4504.02,5132.23");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=325.43,5002.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2520.06,4815.51");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3720.74,4935.86");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5106.23,4909.83");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5691.15,5006.89");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=124.81,4647.29");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1419.78,4751.08");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1892.87,4580.91");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3014.24,4642.91");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4051.27,4729.90");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4712.93,4636.34");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=571.56,4430.99");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=944.77,4523.84");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2304.24,4511.88");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2788.37,4360.25");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3444.00,4486.64");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3944.15,4391.79");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5072.13,4421.52");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5544.36,4469.97");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=807.72,4057.49");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1500.03,4236.54");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2189.92,4062.31");
    }

}

