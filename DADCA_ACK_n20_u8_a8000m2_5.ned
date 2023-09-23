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

network DADCA_ACK_n20_u8_a8000m2_5
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
        @display("i=misc/sensor;p=304.75,7877.59");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=887.77,7869.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1267.64,7843.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1991.58,7925.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2874.40,7866.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3594.37,7911.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3985.69,7775.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4828.49,7994.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5220.10,7992.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1960.86,7556.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2413.18,7709.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4859.28,7633.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5624.70,7517.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=412.80,7423.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=818.91,7466.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1614.78,7452.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2451.34,7295.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3123.56,7336.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3506.01,7295.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4166.67,7472.08");
    }

}

