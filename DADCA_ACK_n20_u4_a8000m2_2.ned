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

network DADCA_ACK_n20_u4_a8000m2_2
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
        @display("i=misc/sensor;p=68.48,7768.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=811.51,7765.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1203.32,7937.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1851.91,7775.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2854.40,7850.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4738.48,7975.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5145.71,7929.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5838.45,7966.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1472.80,7592.40");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2181.46,7610.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2537.98,7569.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3481.54,7745.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4924.51,7513.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5471.61,7626.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5914.55,7557.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1774.49,7267.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3165.97,7310.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4511.05,7426.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=91.51,7174.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=451.40,7116.42");
    }

}

