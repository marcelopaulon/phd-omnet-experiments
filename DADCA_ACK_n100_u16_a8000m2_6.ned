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
        @display("i=misc/sensor;p=776.98,7970.41");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1649.61,7962.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2024.18,7827.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2506.33,7958.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3108.96,7905.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4360.97,7976.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4929.47,7906.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=323.25,7727.79");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1272.03,7736.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2700.50,7604.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3496.61,7693.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4795.35,7578.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5246.13,7751.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5868.84,7595.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=691.34,7315.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1404.16,7330.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2382.08,7343.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3055.51,7432.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4433.00,7480.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5230.00,7327.68");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=41.50,7229.74");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1079.94,7166.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1899.92,7175.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3662.37,7091.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4089.24,7160.70");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4452.14,7014.30");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4877.09,7067.25");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5805.11,7100.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=604.31,6896.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1584.90,6978.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2464.32,6947.60");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3066.47,6835.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4095.48,6791.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5279.60,6852.46");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1148.83,6594.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1948.09,6745.23");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2280.46,6558.21");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3736.75,6631.09");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4458.10,6579.26");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5694.37,6708.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=245.42,6354.99");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1736.38,6339.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2695.38,6424.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3451.25,6317.67");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5150.69,6436.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5503.14,6414.20");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5980.08,6367.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3073.25,6224.36");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4088.49,6174.86");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4699.63,6212.38");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5730.42,6105.92");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=207.18,5957.36");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=737.18,6000.11");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1384.59,5923.19");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1884.34,5828.24");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2276.45,5946.44");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2779.23,5968.88");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3621.05,6003.94");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4293.39,5853.22");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=989.69,5579.57");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1526.54,5564.31");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2231.58,5593.47");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3161.27,5605.91");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3627.07,5569.06");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4919.48,5771.33");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5341.59,5634.88");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5964.33,5715.14");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=361.60,5450.55");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2800.23,5464.88");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4000.44,5393.30");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4407.27,5380.93");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5950.05,5318.35");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1140.19,5260.84");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2565.79,5147.45");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3278.60,5238.05");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3710.57,5193.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4747.97,5265.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5487.59,5275.52");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=295.49,4905.88");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=703.63,4947.24");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1104.27,4839.80");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1522.35,5005.69");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2763.54,4804.57");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3262.41,4835.14");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3820.40,4856.64");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4317.01,4937.30");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5155.25,4972.73");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5554.93,4871.04");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1754.03,4632.95");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2213.05,4753.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4175.11,4594.11");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4742.98,4547.90");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=603.05,4381.95");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=968.60,4305.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1400.06,4531.22");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2746.14,4309.55");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3129.69,4383.41");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3553.62,4532.33");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5975.37,4317.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=315.56,4104.60");
    }

}

