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

network DADCA_ACK_n20_u16_a8000m2_5
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
        @display("i=misc/sensor;p=482.09,7784.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2111.95,7780.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2934.44,7907.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3599.06,7942.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4275.21,7955.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4641.01,7825.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=39.87,7645.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1718.49,7639.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2547.74,7599.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3355.85,7618.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3972.46,7633.28");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5244.96,7643.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5903.05,7743.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=156.49,7296.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=544.70,7431.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1058.27,7501.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2017.01,7371.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4684.96,7415.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5750.18,7341.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=759.03,7111.81");
    }

}

