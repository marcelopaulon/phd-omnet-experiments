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

network DADCA_ACK_n20_u2_a8000m2_2
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
        @display("i=misc/sensor;p=184.23,7754.55");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=686.50,7896.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1688.61,7905.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2357.47,7871.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2766.15,7761.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3404.00,7923.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4418.54,7903.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4942.10,7842.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5601.51,7838.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1210.36,7613.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1882.86,7559.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3401.49,7554.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3778.70,7516.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5292.03,7606.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=741.34,7292.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2999.06,7290.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4684.26,7443.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5931.20,7472.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1120.22,7086.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1603.07,7247.24");
    }

}

