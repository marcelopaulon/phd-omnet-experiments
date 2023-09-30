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

network DADCA_ACK_n100_u16_a8000m2_5
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
        @display("i=misc/sensor;p=662.99,7757.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3947.08,7914.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5379.25,7764.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5780.95,7960.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=48.40,7551.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1194.73,7635.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1867.77,7716.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2486.17,7652.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3330.95,7621.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3977.50,7564.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4482.94,7587.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4859.52,7575.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=618.22,7297.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2356.98,7288.19");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5599.23,7285.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5984.72,7490.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1485.02,7030.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2002.23,7064.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3402.48,7174.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3761.17,7145.09");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4450.16,7159.96");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5055.21,7074.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=618.04,6934.29");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1109.63,7008.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2662.97,6950.59");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4812.90,6815.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5284.44,6807.85");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5949.49,6826.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=361.39,6558.76");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1899.65,6554.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3024.31,6656.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3716.55,6639.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4086.12,6601.85");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4436.78,6543.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=22.75,6444.32");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=902.05,6352.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3393.04,6399.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5342.35,6340.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2553.80,6247.49");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4905.71,6261.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5770.63,6226.47");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=250.85,5957.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=660.28,6011.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1275.10,5968.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1860.44,5911.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2216.93,5996.65");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2654.18,5809.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3194.48,5997.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3833.55,5997.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4348.53,5964.65");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5360.91,5842.32");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=410.19,5531.64");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2973.26,5579.53");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3764.57,5588.46");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4799.38,5751.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5816.12,5762.53");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1070.28,5482.77");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1641.34,5364.06");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2034.43,5291.02");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2371.79,5475.31");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4372.47,5278.71");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4822.92,5343.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5572.19,5496.52");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1135.64,5106.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3014.49,5071.44");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3598.44,5038.68");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5170.21,5060.71");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5852.65,5241.69");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=254.35,4837.46");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=766.39,5007.35");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2503.93,4896.58");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4792.70,4813.16");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5728.41,4851.05");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=812.51,4582.65");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1957.46,4657.37");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2692.23,4545.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3023.81,4720.02");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4116.57,4540.31");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5136.98,4665.52");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=239.19,4357.96");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1443.18,4534.67");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3330.70,4360.69");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3732.99,4444.04");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4872.45,4368.46");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5786.26,4317.89");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=817.83,4065.72");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1229.51,4209.89");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1900.53,4231.11");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2441.66,4256.79");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2856.85,4097.79");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4229.46,4112.34");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5437.18,4281.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=343.03,3878.94");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1457.70,3906.41");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3268.74,3951.73");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3660.97,3816.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4789.36,4020.87");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1065.75,3775.87");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2171.70,3671.25");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2535.69,3696.39");
    }

}

