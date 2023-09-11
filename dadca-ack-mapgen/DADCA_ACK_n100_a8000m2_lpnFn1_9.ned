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

network DADCA_ACK_n100_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=47.36,25.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=56.70,155.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=6.45,90.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=36.57,271.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=50.41,369.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=121.74,373.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=92.25,454.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=24.55,478.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=82.92,537.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=147.89,153.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=181.11,85.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=269.71,18.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=324.82,85.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=350.71,150.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=273.07,131.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=6.02,189.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=162.12,13.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=112.08,266.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=166.44,222.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=231.61,264.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=183.82,478.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=188.85,414.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=201.14,331.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=274.83,353.59");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=341.55,267.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=414.89,209.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=439.04,123.47");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=524.55,102.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=596.48,75.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=515.51,24.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=448.66,17.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=255.81,449.18");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=334.32,387.50");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=380.82,322.97");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=389.10,61.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=441.05,291.08");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=673.09,43.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=710.12,117.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=748.83,42.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=775.39,136.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=624.75,171.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=615.92,252.01");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=547.58,285.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=474.83,172.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=530.31,206.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=269.05,193.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=486.39,342.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=497.23,431.62");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=561.59,453.39");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=552.72,388.56");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=620.79,342.99");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=705.75,376.13");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=776.37,380.68");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=681.33,493.91");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=445.43,399.70");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=398.65,456.15");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=323.97,449.72");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=210.12,539.95");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=143.50,631.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=59.01,629.04");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=17.05,674.57");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=145.51,534.69");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=129.47,695.00");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=208.73,713.33");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=160.83,777.49");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=297.75,576.24");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=570.37,513.88");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=508.66,511.61");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=434.09,558.04");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=366.74,511.09");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=707.00,182.69");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=783.33,208.80");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=803.42,11.70");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=862.68,23.84");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=942.97,37.08");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1032.17,55.98");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1053.27,144.06");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1040.66,218.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1129.71,257.39");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=972.78,170.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=936.24,220.69");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=891.48,147.75");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=686.49,290.10");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=793.49,288.38");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=620.62,6.32");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=755.88,550.70");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=709.50,609.69");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=769.06,641.25");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=822.55,601.55");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=886.29,545.31");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=795.60,438.56");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=275.08,675.20");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=234.31,794.76");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=157.17,867.62");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=218.35,873.88");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=295.30,850.75");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=157.04,944.69");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=108.89,817.75");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=18.11,778.26");
    }

}

