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
        @display("i=misc/sensor;p=1171.95,7962.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2551.94,7876.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4048.58,7998.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4664.18,7974.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5312.75,7859.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5831.16,7825.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=665.59,7628.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1824.84,7599.20");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2658.49,7527.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3141.76,7722.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3627.73,7646.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4187.09,7646.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1379.12,7369.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2301.64,7415.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3938.38,7322.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4569.38,7345.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5024.01,7431.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5831.09,7405.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=684.64,7093.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1064.20,7093.64");
    }

}

