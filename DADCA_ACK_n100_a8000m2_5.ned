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
        @display("i=misc/sensor;p=2502.53,7752.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2966.21,7917.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3313.45,7972.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3787.82,7985.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4482.94,7810.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=290.60,7749.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=947.11,7649.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1578.60,7680.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2175.32,7573.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3250.11,7541.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3599.83,7654.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5421.59,7688.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5870.30,7608.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=651.84,7259.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3737.91,7299.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4141.93,7388.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4681.21,7465.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5614.13,7337.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1303.64,7178.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1848.75,7086.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2796.65,7252.37");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3288.08,7080.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5176.82,7200.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=390.16,6936.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=949.57,6899.60");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2297.79,6929.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2664.43,6763.05");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3610.42,6938.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4250.32,6800.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4644.18,6907.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5581.76,6788.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5962.14,6818.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1313.11,6634.70");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2022.47,6672.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2962.46,6550.03");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=169.21,6478.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=818.52,6410.81");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1717.57,6491.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2143.05,6343.55");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3887.05,6322.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4898.53,6294.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5365.27,6429.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=348.89,6038.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1280.88,6264.12");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2614.22,6170.30");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3205.14,6041.78");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5946.10,6133.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1229.40,5807.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1706.05,5929.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2078.69,5922.10");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2494.27,5830.11");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2889.52,5882.15");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3956.65,5876.57");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4488.07,5995.12");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5262.16,6006.33");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=731.31,5719.64");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3199.06,5574.43");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3592.28,5738.83");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5198.20,5635.72");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5836.38,5745.72");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1456.42,5407.39");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1932.12,5327.40");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2717.19,5436.43");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4270.39,5407.77");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4823.83,5447.45");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5461.86,5348.51");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5905.42,5336.25");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=602.77,5126.12");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1184.93,5077.41");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3105.78,5173.73");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3518.87,5116.84");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4870.43,5073.54");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=231.55,4890.06");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1547.09,4925.66");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2133.25,4929.47");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2517.95,4951.91");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3969.68,4961.97");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4448.98,4990.36");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5676.62,5003.06");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=826.43,4700.89");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1182.51,4719.16");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1857.33,4703.71");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3529.46,4749.80");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3935.09,4580.79");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4582.82,4559.31");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5243.26,4668.05");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=508.41,4291.09");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2320.50,4344.72");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2792.55,4492.20");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3180.77,4460.19");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3637.61,4369.07");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4898.94,4357.47");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5796.88,4362.10");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=156.83,4122.58");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1308.42,4101.95");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1823.92,4219.16");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3864.47,4064.29");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4254.74,4191.76");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5452.86,4219.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2615.93,4011.91");
    }

}

