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
        @display("i=misc/sensor;p=27.11,7849.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=380.93,7914.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1543.56,7995.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2254.31,7920.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3030.90,7902.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3797.38,7891.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5587.31,7785.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1098.41,7646.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1696.28,7628.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2694.09,7642.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3349.34,7551.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4079.86,7637.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5054.55,7534.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=449.03,7390.44");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1402.85,7312.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1957.85,7370.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2362.22,7463.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4447.85,7420.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3.87,7168.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1039.27,7229.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2420.00,7078.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3632.96,7250.99");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5278.84,7024.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5810.60,7155.49");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=241.69,6838.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=906.68,6788.56");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1418.71,6910.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2803.11,6978.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3248.38,6917.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4087.00,6853.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4868.98,6943.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1546.24,6542.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2118.11,6727.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3649.72,6722.91");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4410.46,6568.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4866.90,6533.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5496.17,6561.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=395.92,6460.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1232.22,6350.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2143.55,6333.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2531.54,6387.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2882.58,6496.61");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3424.94,6276.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4136.65,6282.75");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=906.74,6091.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5181.74,6229.48");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5554.76,6153.55");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=436.65,5927.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1379.15,5839.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1759.35,5991.05");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3183.34,5896.84");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4537.36,5969.64");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5036.94,5855.81");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5394.76,5779.78");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=716.77,5643.86");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2288.58,5646.25");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3907.66,5737.28");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4360.08,5632.94");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5964.74,5756.60");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1754.15,5291.65");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2679.29,5379.54");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3095.49,5477.33");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5809.15,5323.73");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=590.58,5272.98");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1174.90,5186.38");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2408.30,5097.85");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3511.85,5236.66");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3894.13,5249.13");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4420.98,5193.18");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4877.74,5138.38");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5332.49,5109.77");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=66.92,4810.40");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1389.39,4896.19");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1773.64,4881.93");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3003.80,5029.96");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4269.58,4852.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5979.84,5017.81");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=991.48,4761.45");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2568.42,4567.64");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3176.54,4650.42");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3710.09,4571.70");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4719.11,4723.53");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5206.49,4648.43");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5604.21,4649.77");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=318.01,4522.60");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1169.62,4439.55");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1861.52,4497.82");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2919.90,4401.64");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4400.96,4491.82");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=557.55,4263.42");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1650.50,4195.69");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2445.43,4208.79");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3245.95,4217.20");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3597.37,4222.06");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4694.35,4093.57");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5044.22,4204.67");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5919.63,4198.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=238.90,4020.97");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=683.32,3795.07");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1048.76,3878.70");
    }

}

