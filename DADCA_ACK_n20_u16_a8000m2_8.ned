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

network DADCA_ACK_n20_u16_a8000m2_8
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
        @display("i=misc/sensor;p=294.11,7905.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=647.66,7846.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1247.66,7834.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1629.84,7802.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1987.29,7987.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2597.74,7817.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3644.12,7930.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3998.83,7955.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4652.41,7984.74");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5606.85,7775.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1851.62,7528.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2925.37,7558.63");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3736.46,7555.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4413.24,7650.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5064.32,7718.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=19.45,7404.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=713.36,7358.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1057.52,7432.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1425.14,7406.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2301.26,7283.81");
    }

}

