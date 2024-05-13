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

network DADCA_ACK_n50_u2_a8000m2_5
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
        @display("i=misc/sensor;p=777.51,7868.55");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1839.27,7768.33");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3205.59,7806.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4049.80,7764.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4470.43,7990.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4904.46,7976.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5213.82,7774.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=615.87,7514.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1366.00,7730.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2632.40,7737.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3629.11,7630.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4739.38,7594.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5706.13,7602.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=192.90,7399.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1531.47,7415.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2026.80,7271.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2426.59,7326.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3044.97,7282.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3978.55,7280.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5141.86,7427.43");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=902.91,7240.21");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3567.82,7071.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4474.12,7182.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5489.95,7040.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5780.92,7244.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=349.19,6984.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1297.34,6789.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2325.68,6935.28");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2798.60,6970.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4094.27,6896.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5038.70,6988.64");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=192.93,6605.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=548.09,6581.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=958.22,6632.78");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1910.05,6705.04");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2413.18,6595.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2902.28,6609.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3381.67,6532.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3839.39,6601.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4729.85,6636.85");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5670.05,6642.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=785.40,6281.13");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1612.56,6492.60");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3665.12,6283.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4325.87,6388.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5296.57,6440.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=240.74,6201.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1350.94,6194.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1905.28,6215.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2316.62,6245.25");
    }

}

