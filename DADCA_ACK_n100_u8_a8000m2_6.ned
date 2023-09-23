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

network DADCA_ACK_n100_u8_a8000m2_6
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
        @display("i=misc/sensor;p=298.42,7862.65");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=724.50,7789.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1866.91,7832.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2327.11,7814.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3018.49,7889.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3581.37,7991.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4738.99,7848.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5589.48,7840.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1231.07,7714.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2792.36,7582.83");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3623.54,7536.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4266.26,7679.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5133.03,7662.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=247.88,7501.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=948.11,7470.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2282.60,7466.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4690.67,7366.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=545.72,7073.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1393.42,7204.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1850.49,7184.39");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2517.15,7031.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3229.34,7203.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3730.47,7185.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4201.52,7162.33");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5385.54,7256.34");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2931.22,6992.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4997.59,6939.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1208.80,6529.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1577.09,6603.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1972.04,6656.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2904.20,6549.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3743.89,6705.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4178.76,6675.21");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4572.49,6759.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5194.13,6594.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5782.43,6560.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=20.29,6435.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=442.49,6446.80");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2278.14,6373.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3341.89,6438.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4146.91,6313.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4606.34,6353.52");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1438.91,6021.39");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1756.36,6207.46");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2907.20,6195.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3426.48,6079.28");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4957.51,6031.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=327.91,5930.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=812.23,5996.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2503.74,5807.91");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4392.73,5782.50");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5501.37,6003.90");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5947.12,5789.81");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1355.33,5681.43");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2074.14,5755.42");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2996.19,5765.49");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3586.70,5712.86");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5271.33,5677.10");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=213.49,5430.17");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=796.63,5300.20");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1160.34,5296.75");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3223.69,5483.31");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3833.39,5410.72");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4962.19,5339.66");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5537.53,5427.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5909.10,5341.32");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1660.98,5155.21");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2189.52,5216.87");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2705.52,5235.77");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3013.10,5038.77");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3535.56,5106.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4411.95,5269.46");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=778.98,4922.56");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1232.83,4821.10");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2168.17,4814.02");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3896.95,4871.10");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4829.93,4816.37");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5205.14,4827.29");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5610.51,4919.77");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=299.26,4709.47");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1669.07,4781.48");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3285.32,4766.39");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4291.25,4719.98");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5864.93,4668.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1001.88,4297.54");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2185.85,4371.07");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2756.26,4413.42");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3785.52,4322.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4709.12,4379.83");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5485.08,4534.92");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=120.74,4283.68");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=456.50,4124.92");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1563.36,4280.71");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3038.32,4133.12");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4286.94,4242.11");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5139.33,4243.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5694.74,4109.54");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=879.68,3937.29");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2185.33,3946.99");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2583.30,3864.44");
    }

}

