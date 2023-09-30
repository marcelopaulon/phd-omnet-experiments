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

network DADCA_ACK_n100_u16_a8000m2_3
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
        @display("i=misc/sensor;p=733.90,7835.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1999.82,7952.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2738.93,7930.75");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3901.06,7959.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4328.02,7951.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5078.19,7999.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5536.73,7886.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=380.52,7563.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1120.92,7545.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1706.03,7697.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3145.91,7695.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3823.89,7555.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4970.87,7664.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5940.93,7518.38");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=723.51,7452.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1828.78,7301.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2217.39,7438.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2632.66,7375.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3460.78,7337.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4435.76,7363.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5385.39,7326.91");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=92.63,7221.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=456.96,7118.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1311.69,7184.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2080.91,7046.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2503.89,7042.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2894.28,7050.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5866.45,7158.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=728.50,6896.90");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3253.22,6978.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3876.66,6945.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4785.11,6798.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5341.76,6955.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=188.11,6705.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1435.06,6701.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1940.81,6672.87");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2663.44,6545.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3707.78,6547.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4294.25,6753.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5497.59,6580.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5918.29,6724.79");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=697.97,6512.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1057.74,6417.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1916.38,6307.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3384.11,6407.88");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5134.94,6288.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5847.84,6364.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1410.83,6222.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2545.07,6202.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3020.34,6155.45");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3693.62,6163.26");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4083.26,6057.97");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4732.86,6174.53");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5566.65,6147.66");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=764.67,5927.79");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1771.82,5899.40");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2280.21,5834.02");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2664.16,5788.71");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3428.60,5827.69");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4329.94,5793.53");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5262.21,5819.96");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5862.61,5924.54");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1145.07,5729.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1520.19,5536.23");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3964.09,5596.29");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4784.28,5608.82");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=545.60,5438.16");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1206.06,5360.98");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2161.15,5304.93");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2489.64,5452.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3201.07,5524.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3625.70,5403.40");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5415.27,5475.43");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5772.67,5371.60");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1757.74,5258.81");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3662.70,5046.40");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4108.00,5239.78");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4537.44,5230.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4917.57,5244.90");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=260.51,4813.30");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=677.38,5023.23");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1327.09,4908.19");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2372.85,4843.84");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2744.11,5013.44");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3144.57,5027.60");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4915.82,4817.09");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5295.71,4984.35");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5840.04,4964.51");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=751.11,4568.28");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1774.78,4758.76");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3004.00,4570.04");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3630.18,4595.13");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4091.07,4576.13");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5584.18,4716.04");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4470.49,4372.92");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4994.35,4474.92");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5540.93,4351.11");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=928.66,4233.08");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1311.02,4230.98");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1968.03,4273.09");
    }

}

