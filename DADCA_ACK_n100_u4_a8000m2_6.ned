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

network DADCA_ACK_n100_u4_a8000m2_6
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
        @display("i=misc/sensor;p=1579.49,7833.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3051.20,7845.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4729.76,7919.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5372.54,7929.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=995.43,7659.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2220.77,7658.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2654.16,7671.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3455.99,7637.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4149.90,7640.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5111.32,7541.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1167.29,7303.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2093.62,7282.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3114.25,7476.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4532.39,7399.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=102.58,7208.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=660.38,7185.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1717.43,7251.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3406.00,7178.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4076.52,7095.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4677.66,7025.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5174.27,7137.33");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=503.63,6816.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1493.41,6863.65");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1973.92,6796.11");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2563.70,6933.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3741.51,6930.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5024.79,6813.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5751.85,6912.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=121.58,6602.16");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=965.20,6651.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2946.75,6668.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3310.58,6559.66");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4197.74,6686.44");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5420.82,6631.26");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=634.04,6329.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1542.28,6452.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2263.32,6270.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2809.98,6345.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3913.53,6406.36");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4540.55,6415.43");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5019.42,6442.96");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5695.27,6413.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1072.47,6162.75");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1477.70,6024.85");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1800.01,6179.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3270.90,6137.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4213.81,6196.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5086.57,6041.06");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=221.30,5795.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1192.43,5803.23");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2101.02,5844.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2541.77,6020.07");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3732.31,5820.01");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4571.10,6019.22");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5542.22,5868.80");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=764.13,5690.63");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1568.09,5526.06");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3289.37,5760.80");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4392.62,5679.97");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5323.39,5578.48");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=179.48,5447.55");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=617.57,5326.49");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2291.40,5428.76");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3498.09,5360.91");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4299.64,5277.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4894.07,5515.22");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5716.20,5391.92");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1125.64,5245.99");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2823.17,5155.26");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3188.16,5124.83");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3947.88,5155.00");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4757.94,5130.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5153.24,5092.45");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5514.68,5061.03");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=333.53,4963.57");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=879.28,4871.24");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1286.50,4873.99");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1845.12,4898.89");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2510.94,4811.27");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3025.82,4611.77");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3660.63,4774.87");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4304.95,4644.86");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5247.34,4718.91");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5766.30,4723.04");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=177.37,4523.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=757.53,4518.25");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2460.56,4305.37");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3971.13,4447.41");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4951.41,4530.71");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5310.40,4314.89");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=915.96,4059.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1300.39,4265.74");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1985.91,4233.14");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3531.44,4274.85");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4014.57,4095.06");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4511.82,4206.09");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5018.85,4086.61");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5988.77,4165.05");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=544.66,3996.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1488.12,3911.43");
    }

}

