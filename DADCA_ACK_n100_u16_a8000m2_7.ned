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

network DADCA_ACK_n100_u16_a8000m2_7
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
        @display("i=misc/sensor;p=800.30,7940.65");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1298.62,7831.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2037.87,7753.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2719.06,7907.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4853.03,7908.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5615.13,7832.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1054.41,7510.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1495.77,7527.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2383.03,7665.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3500.47,7658.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4004.78,7522.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4312.48,7731.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=67.98,7388.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=467.22,7349.76");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2956.90,7413.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4626.69,7423.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5176.41,7459.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1357.48,7203.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1783.48,7148.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2228.17,7113.03");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2623.85,7016.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5582.40,7213.36");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=13.29,6985.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=505.40,6967.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=831.18,6779.22");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3233.52,6785.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3729.85,6946.83");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4353.75,6921.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5149.56,6972.22");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=256.03,6613.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1174.69,6558.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1782.04,6662.26");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2215.22,6558.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2636.19,6579.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4020.45,6731.35");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5023.17,6569.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5584.40,6706.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=470.52,6325.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=819.42,6297.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3841.48,6388.43");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4307.34,6358.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5833.28,6291.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=108.81,6042.76");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1562.34,6210.11");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2079.26,6232.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2699.71,6167.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3129.85,6179.55");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4783.24,6169.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5236.19,6117.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=522.94,5925.89");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1299.34,5901.50");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2387.73,5854.65");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3422.59,5843.19");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3938.43,5945.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4900.06,5788.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5645.53,5947.31");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=226.73,5692.95");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1274.43,5544.14");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1806.46,5599.93");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4258.67,5599.56");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5376.51,5682.66");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=864.61,5517.17");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2190.79,5500.20");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2982.86,5458.66");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3874.17,5299.10");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5961.31,5411.33");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=434.19,5098.38");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1465.49,5114.02");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2644.42,5265.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3031.84,5093.13");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4399.31,5043.69");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4694.14,5258.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5292.27,5156.02");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=891.38,4965.60");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1818.69,4904.74");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2389.25,4956.40");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3561.31,4968.84");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4957.99,4984.33");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5660.48,4842.39");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4034.51,4710.56");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4718.87,4643.96");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5268.44,4719.66");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=275.03,4427.42");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=813.00,4503.90");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1226.72,4522.05");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1876.05,4512.61");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2521.58,4462.36");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3144.63,4394.42");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3889.94,4372.77");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4992.21,4386.13");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5551.81,4374.64");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=180.69,4088.62");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=712.16,4052.85");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1084.21,4077.36");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1528.06,4091.26");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2118.11,4121.52");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3561.63,4235.37");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4675.69,4053.27");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2359.36,3863.00");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2694.48,4005.02");
    }

}

