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

network DADCA_ACK_n50_u8_a8000m2_3
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
        @display("i=misc/sensor;p=1026.02,7918.58");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1426.72,7823.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1826.55,7798.33");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3170.62,7899.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3557.98,7915.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4755.73,7892.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2221.71,7596.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2662.38,7627.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4035.10,7703.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=969.89,7298.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1427.64,7285.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1777.36,7411.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2494.92,7309.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3083.59,7412.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3721.27,7498.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4743.92,7468.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5329.27,7381.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=507.91,7109.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2155.75,7191.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3640.56,7040.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4223.63,7247.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1098.36,6809.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2776.56,6863.06");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4670.03,6806.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5129.01,6922.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5812.50,6962.73");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=309.80,6690.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=726.00,6536.66");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1781.08,6761.24");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2150.46,6720.54");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2512.30,6611.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4123.55,6591.92");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5212.84,6580.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=337.15,6317.11");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=985.68,6276.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1323.28,6395.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2009.74,6271.39");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2946.65,6440.41");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4635.44,6310.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5801.00,6328.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2436.73,6214.15");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3413.46,6023.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4104.86,6150.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5006.89,6100.62");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=693.12,5857.09");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1291.40,6015.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1726.71,5926.40");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2161.88,5805.86");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3062.05,5892.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4396.35,5874.31");
    }

}

