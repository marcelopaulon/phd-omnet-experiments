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

network DADCA_ACK_n50_u8_a8000m2_7
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
        @display("i=misc/sensor;p=554.72,7860.05");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1101.46,7924.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1741.08,7942.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3327.40,7797.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4802.26,7909.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5329.92,7935.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=224.75,7678.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1299.71,7574.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1957.87,7605.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2402.29,7595.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2920.78,7710.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3563.95,7517.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3975.39,7646.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4476.12,7597.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4822.25,7530.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2698.74,7373.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4124.29,7279.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5718.72,7365.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=95.73,7034.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=616.49,7164.60");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1494.25,7220.51");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2074.91,7086.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3465.44,7174.48");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4401.74,7049.56");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5071.75,7212.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1056.65,6802.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1743.85,6904.33");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2922.62,6923.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3966.77,6815.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5852.62,6847.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=595.73,6617.86");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2061.68,6585.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2503.21,6550.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3045.28,6586.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4466.61,6686.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4919.22,6716.17");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=128.12,6497.86");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=906.23,6423.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1265.92,6463.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1588.48,6287.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2817.72,6271.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3691.71,6504.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4882.73,6327.96");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5417.54,6482.15");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5818.14,6420.44");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=980.83,6048.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3424.72,6105.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4099.15,6178.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4665.69,6034.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5102.50,6054.12");
    }

}

