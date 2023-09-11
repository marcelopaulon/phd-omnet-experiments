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

network DADCA_ACK_n100_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=157.48,48.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=70.65,10.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=246.58,21.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=255.11,92.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=174.04,125.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=60.57,156.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=140.83,211.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=193.32,264.39");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=136.34,331.10");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=197.78,348.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=97.26,270.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=69.70,345.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=252.92,434.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=177.08,493.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=67.32,472.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=66.97,405.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=46.95,573.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=90.98,615.73");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=59.45,710.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=60.57,774.78");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=104.42,831.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=65.46,889.87");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=31.88,981.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=165.32,986.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=255.04,953.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=223.29,899.92");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=201.76,832.78");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=181.03,757.09");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=141.42,708.78");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=156.50,643.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=125.25,423.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=277.90,308.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=294.92,382.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=407.83,434.69");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=462.28,472.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=471.02,540.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=562.63,533.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=553.80,435.01");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=566.99,358.86");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=645.16,333.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=501.36,304.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=476.25,214.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=393.58,216.91");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=537.44,222.51");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=349.49,127.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=302.30,173.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=312.59,26.87");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=369.81,62.23");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=433.95,2.61");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=556.74,22.34");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=632.97,65.77");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=623.28,150.61");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=537.42,121.08");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=747.31,59.91");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=677.63,119.00");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=632.29,212.20");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=649.36,451.30");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=711.04,458.12");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=621.99,596.80");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=568.76,663.81");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=717.54,644.10");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=792.44,647.67");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=793.62,722.98");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=852.90,667.30");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=904.74,728.60");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=931.88,641.22");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=405.36,598.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=354.27,681.22");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=288.69,673.44");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=319.61,740.80");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=467.97,722.83");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=473.06,643.45");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=335.93,619.78");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=244.36,738.04");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=353.05,825.23");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=373.96,885.36");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=452.63,845.50");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=526.10,819.15");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=599.97,867.50");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=647.08,911.46");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=499.59,894.70");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=479.11,967.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=557.49,1002.60");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=598.19,1057.45");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=659.68,980.43");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=379.54,983.06");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=416.51,1037.27");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=420.95,1112.00");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=344.80,1047.81");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=266.68,817.96");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=244.07,606.70");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=296.73,539.90");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=376.45,501.72");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=425.98,366.44");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=368.12,319.65");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=487.69,366.18");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=441.15,305.27");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=669.96,508.22");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=822.02,562.48");
    }

}

