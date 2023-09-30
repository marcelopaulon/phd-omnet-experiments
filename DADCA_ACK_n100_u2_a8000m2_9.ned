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

network DADCA_ACK_n100_u2_a8000m2_9
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
        @display("i=misc/sensor;p=241.70,7974.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=706.78,7858.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1108.05,7837.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1906.57,7919.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3481.03,7950.71");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3908.69,7938.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4588.11,7821.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5131.97,7907.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5999.31,7916.26");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=300.17,7592.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1580.28,7708.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2383.99,7678.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2747.94,7667.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3197.57,7705.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4220.42,7618.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5414.61,7632.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=35.44,7320.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=686.11,7268.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1169.93,7377.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1828.64,7332.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2967.60,7320.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3593.27,7343.28");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5212.60,7340.15");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5862.02,7480.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2235.34,7108.27");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2634.15,7049.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3267.90,7027.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4281.53,7029.30");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4691.88,7215.62");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5599.13,7028.08");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=325.38,6935.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1561.21,6906.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5014.64,6796.21");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5948.52,6813.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=918.31,6734.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2090.08,6669.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3111.85,6575.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3628.93,6548.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3939.79,6725.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4329.02,6678.69");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5520.23,6542.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=22.35,6456.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=445.76,6346.13");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1199.64,6324.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1571.43,6376.72");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2629.74,6452.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4198.34,6347.51");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4850.47,6374.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2423.77,6110.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3057.65,6042.46");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3577.88,6059.56");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5946.38,6113.02");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=117.36,5974.19");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=824.49,5870.53");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2657.87,5848.01");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4564.20,5874.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5032.72,5980.20");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=535.39,5639.23");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1440.84,5656.69");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1860.49,5744.21");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2995.45,5612.59");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3417.70,5572.67");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3984.78,5735.13");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1869.87,5322.66");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2710.66,5398.00");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3842.08,5286.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4300.15,5408.40");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4755.16,5370.61");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1113.29,5260.68");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2423.15,5159.96");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3448.24,5039.26");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=647.08,4969.58");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1992.66,4867.09");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3111.12,4941.62");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3884.55,4901.26");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=912.50,4702.90");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1583.90,4738.00");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2381.69,4722.96");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3597.83,4663.34");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4253.71,4645.50");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4870.27,4581.30");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5481.07,4761.37");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=374.41,4363.85");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1396.25,4413.16");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1845.40,4299.61");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3005.05,4493.72");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5701.48,4372.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=161.38,4058.91");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=743.61,4184.34");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2499.19,4099.66");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3065.41,4041.43");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3551.55,4061.22");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3952.65,4235.22");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4390.79,4237.34");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4948.75,4053.46");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5294.74,4189.46");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5993.26,4142.14");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=760.50,3825.28");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1244.48,3931.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1935.10,3929.95");
    }

}

