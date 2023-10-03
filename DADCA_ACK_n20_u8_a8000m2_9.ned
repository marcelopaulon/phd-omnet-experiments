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

network DADCA_ACK_n20_u8_a8000m2_9
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
        @display("i=misc/sensor;p=58.21,7765.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2223.15,7997.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2681.92,7843.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3597.98,7985.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4574.09,7752.71");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5128.54,7948.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5623.30,7855.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=571.09,7643.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1300.32,7562.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1734.67,7718.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2250.03,7644.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3745.23,7658.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4227.04,7574.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5960.55,7729.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2797.19,7377.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3261.69,7438.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4731.91,7416.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5125.71,7473.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5617.29,7484.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=253.64,7171.78");
    }

}

