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

network DADCA_ACK_n100_u2_a8000m2_3
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
        @display("i=misc/sensor;p=929.17,7971.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1327.00,7972.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2399.65,7796.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3753.03,7869.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4635.36,7882.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5473.82,7874.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=428.99,7664.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=909.49,7619.33");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1623.90,7590.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2034.47,7724.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2812.21,7569.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3205.29,7685.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5046.93,7719.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5725.52,7601.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=477.04,7261.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2104.66,7288.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3077.16,7258.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4274.92,7459.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4743.66,7470.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=828.45,7032.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2681.68,7040.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3653.95,7194.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4841.62,7132.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5272.85,7248.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5756.78,7154.22");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=222.04,6993.75");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1814.30,6928.32");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2258.61,6827.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3349.00,6851.16");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3702.06,6786.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4097.57,6890.28");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5187.96,6856.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=76.29,6567.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1446.05,6725.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2683.11,6522.18");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3131.15,6575.17");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=505.08,6429.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1144.59,6408.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2060.77,6413.51");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3486.30,6408.68");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3826.28,6286.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4187.34,6312.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4569.80,6489.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5049.22,6481.49");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5599.67,6324.01");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=166.66,6117.67");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=800.64,6139.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1623.18,6211.83");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2500.28,6152.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=504.60,5913.15");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2032.08,5782.72");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2692.92,5858.31");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3167.02,5940.77");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3609.69,6006.89");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4429.07,5998.88");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4976.09,6018.02");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5365.06,6017.13");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1464.64,5669.31");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3421.51,5627.13");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3922.57,5749.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4781.16,5690.45");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5460.77,5626.06");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=267.84,5522.34");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=644.83,5280.06");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=926.13,5498.56");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2234.67,5409.51");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4314.47,5412.16");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4702.68,5325.54");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=288.44,5139.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1305.73,5264.20");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1868.60,5181.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2992.15,5205.59");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3756.94,5173.66");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5340.60,5275.42");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=557.02,4869.20");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1111.88,4829.50");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2490.89,4968.55");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3358.94,4912.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4287.10,4970.15");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5828.31,5020.84");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=144.85,4746.71");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1625.27,4635.79");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2846.20,4630.86");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4236.36,4560.57");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4691.10,4656.84");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5241.94,4677.63");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5718.37,4576.31");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=171.19,4373.74");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=707.52,4340.46");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1286.56,4429.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2172.30,4469.90");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3851.80,4531.59");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4665.10,4299.19");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1659.85,4256.68");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2575.36,4257.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3116.41,4087.57");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3517.60,4238.60");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4290.60,4201.57");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4995.04,4059.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5437.28,4046.19");
    }

}

