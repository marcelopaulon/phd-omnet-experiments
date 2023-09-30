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

network DADCA_ACK_n20_u3_a8000m2_4
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
        @display("i=misc/sensor;p=379.30,7905.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=765.47,7945.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1453.24,7896.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1894.41,7782.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3781.27,7877.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4784.85,7807.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5977.71,7763.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=682.49,7605.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1116.16,7733.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2462.22,7637.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3510.27,7646.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4283.01,7604.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5484.17,7717.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=280.19,7334.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1483.87,7448.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3002.65,7407.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3339.29,7304.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3940.81,7466.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5301.53,7282.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5986.14,7338.88");
    }

}

