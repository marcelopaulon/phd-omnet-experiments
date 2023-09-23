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

network DADCA_ACK_n20_u2_a8000m2_7
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
        @display("i=misc/sensor;p=149.28,7854.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=669.62,7837.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1677.11,7900.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2920.24,7765.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3350.67,7863.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4374.45,7833.75");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5229.08,7791.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=981.39,7646.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2302.84,7711.40");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3899.55,7599.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4890.30,7620.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5794.66,7713.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=388.40,7267.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1787.90,7430.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2390.33,7263.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2742.56,7443.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3369.09,7500.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5263.47,7337.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5620.26,7329.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1384.08,7167.77");
    }

}

