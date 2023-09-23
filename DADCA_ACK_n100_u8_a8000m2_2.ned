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

network DADCA_ACK_n100_u8_a8000m2_2
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
        @display("i=misc/sensor;p=45.33,7947.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=430.89,7892.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2100.21,7950.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2533.69,7988.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2917.10,7982.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3945.69,7889.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4998.28,7845.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=758.75,7729.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1383.40,7747.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1807.62,7720.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2273.77,7570.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2681.00,7632.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3148.27,7682.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3540.50,7718.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4384.10,7722.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5347.07,7696.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=128.84,7391.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=975.74,7308.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1589.45,7429.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3098.58,7333.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4059.97,7364.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4865.67,7428.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5781.27,7302.90");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=571.80,7214.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1342.82,7033.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1931.23,7019.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2501.97,7218.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3450.33,7068.73");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=206.40,6976.47");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=924.45,6872.85");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2370.21,6810.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4478.95,7002.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5190.90,7006.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5657.77,6840.22");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=641.95,6650.74");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1236.64,6539.14");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1555.47,6755.68");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3157.56,6532.72");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3974.88,6755.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=630.91,6285.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1843.37,6331.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2753.91,6392.54");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3890.13,6360.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4262.12,6359.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4767.20,6491.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5433.72,6423.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5982.30,6349.00");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=183.21,6212.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=995.78,6222.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1474.37,6130.61");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2417.86,6177.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3580.55,6058.24");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4828.08,6035.75");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5208.07,6070.97");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=729.25,5827.38");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1169.04,5916.19");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2020.51,5798.10");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3082.47,5987.59");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3919.85,5920.37");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4386.11,5937.07");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=366.72,5697.48");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1004.12,5526.42");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1686.54,5649.56");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2843.94,5638.59");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4680.79,5559.97");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5737.46,5700.99");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=94.84,5314.31");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2199.79,5487.16");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2634.41,5328.39");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3299.19,5419.02");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3801.06,5300.59");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=487.72,5166.22");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1174.18,5138.81");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1529.90,5094.16");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2042.18,5104.32");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4197.71,5269.31");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5232.98,5261.97");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5996.41,5100.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=17.62,4830.42");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=407.76,4782.67");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=858.65,4974.24");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3161.60,4983.33");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3520.84,4793.80");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4108.43,4831.58");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4727.91,4912.50");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=758.81,4576.41");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1180.25,4780.35");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1607.34,4558.23");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1944.11,4701.83");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2897.60,4665.45");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4555.01,4604.20");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5506.92,4728.68");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5949.07,4563.35");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3538.02,4342.60");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=413.89,4241.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1409.74,4252.67");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1794.25,4053.83");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2426.40,4205.83");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2979.40,4271.62");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4175.54,4148.72");
    }

}

