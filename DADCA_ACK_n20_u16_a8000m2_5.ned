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
        @display("i=misc/sensor;p=786.88,7843.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1185.24,7767.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1935.79,7816.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2817.98,7995.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3203.41,7799.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4037.37,7815.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5320.35,7868.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=200.75,7646.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2285.69,7529.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2625.07,7666.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3723.66,7591.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4969.09,7740.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5902.77,7621.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1158.73,7401.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1717.22,7303.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3039.89,7407.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4075.84,7304.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4436.22,7300.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5560.93,7499.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=673.83,7247.21");
    }

}

