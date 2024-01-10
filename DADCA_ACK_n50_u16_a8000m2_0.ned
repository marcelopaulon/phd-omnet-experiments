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

network DADCA_ACK_n50_u16_a8000m2_0
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
        @display("i=misc/sensor;p=987.51,7778.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2702.30,7829.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3293.73,7936.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4439.42,7838.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5998.83,7777.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=710.96,7559.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1314.72,7602.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1753.74,7682.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2110.94,7533.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3586.39,7733.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4820.14,7705.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5658.45,7580.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=45.00,7380.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=443.44,7303.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2627.80,7444.29");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3097.98,7415.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3549.11,7345.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3897.18,7444.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4514.32,7314.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5065.55,7387.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1315.40,7237.96");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1740.82,7183.47");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2289.78,7180.68");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3282.03,7022.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3954.10,7076.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4911.86,7013.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5778.56,7026.56");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=49.52,6785.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=478.71,6928.81");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=863.26,6957.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1897.28,6805.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2454.90,6835.35");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4409.88,6867.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5411.86,6986.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1062.04,6600.70");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1427.77,6668.10");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3032.39,6731.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3443.08,6692.35");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4106.37,6534.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5695.83,6553.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=650.96,6296.21");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2174.76,6328.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4720.84,6430.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=175.35,6254.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1370.45,6258.69");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2729.41,6253.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3251.94,6076.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3936.49,6045.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5073.46,6174.28");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5531.51,6138.94");
    }

}

