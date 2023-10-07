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
        @display("i=misc/sensor;p=1580.55,7848.48");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1989.52,7896.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3018.22,7821.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3375.51,7858.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3862.59,7962.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4542.57,7909.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5929.45,7973.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1106.13,7636.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1796.84,7543.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2478.35,7533.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4278.76,7629.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5119.31,7698.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=208.59,7489.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2850.46,7349.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3940.64,7461.59");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4729.75,7320.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5614.01,7500.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5999.05,7377.79");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=313.55,7113.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=745.98,7249.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1482.85,7208.63");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3440.24,7187.46");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4241.29,7124.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2086.79,6965.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2483.69,6875.99");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3048.77,6899.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5225.09,6972.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5585.39,6966.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5960.85,6898.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=50.72,6531.79");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=757.52,6711.02");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1433.40,6620.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1814.16,6742.44");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2427.86,6517.64");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3422.43,6705.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3912.34,6737.78");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4569.74,6662.61");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5062.25,6552.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5874.19,6534.52");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=967.68,6412.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2766.52,6394.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3297.03,6331.38");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4255.53,6272.11");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5452.55,6458.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=421.01,6211.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1399.42,6239.39");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1985.88,6200.54");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2927.18,6049.27");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4759.61,6071.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5412.47,6062.24");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5965.95,6099.06");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=24.02,5951.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1199.72,5804.93");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2361.06,5809.26");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3339.15,5820.93");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3885.94,5965.55");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4462.77,5802.66");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5113.58,5789.36");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=838.66,5727.66");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5899.71,5734.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=20.04,5323.64");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1176.04,5442.42");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1805.95,5488.88");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2720.70,5483.81");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3729.50,5326.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4179.26,5470.88");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5626.46,5346.66");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=506.33,5243.23");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1294.30,5065.31");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2270.56,5167.94");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2839.98,5103.02");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4708.80,5231.59");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5134.01,5054.13");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=62.96,4917.93");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1842.80,4870.98");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3983.24,4885.83");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5479.72,4893.66");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5864.06,4969.16");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=269.52,4612.02");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=624.44,4734.51");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1118.18,4724.07");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1456.61,4582.58");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2296.48,4557.32");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3384.10,4760.29");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4293.63,4667.16");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4979.41,4545.08");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1173.49,4328.18");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1897.54,4327.84");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2756.51,4504.10");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3420.97,4375.32");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4639.77,4297.60");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5606.08,4319.42");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=14.35,4148.34");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=757.81,4276.90");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2265.43,4198.89");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3766.76,4252.54");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4165.12,4089.03");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=339.91,3795.84");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=821.22,3926.08");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1425.95,4031.22");
    }

}

