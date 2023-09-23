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

network DADCA_ACK_n20_u4_a8000m2_7
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
        @display("i=misc/sensor;p=380.97,7910.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1568.36,7802.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2310.38,7847.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2671.35,7953.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3570.28,7854.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4160.39,7756.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5206.20,7976.74");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=441.85,7522.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=818.95,7625.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3115.00,7610.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4816.11,7626.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5513.01,7628.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=93.56,7397.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1182.40,7430.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2010.63,7368.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2711.40,7276.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=504.19,7092.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1031.86,7056.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1721.22,7059.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3167.27,7237.33");
    }

}

