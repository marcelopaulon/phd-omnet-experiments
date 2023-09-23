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

network DADCA_ACK_n20_u2_a8000m2_4
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
        @display("i=misc/sensor;p=186.31,7803.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1181.44,7917.03");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1610.31,7871.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2403.14,7978.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4191.27,7942.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1906.38,7519.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2483.66,7625.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3133.75,7526.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3793.34,7519.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4251.07,7552.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5153.62,7694.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1040.88,7387.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4827.77,7441.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5450.04,7373.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5906.36,7312.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=330.45,7243.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1392.84,7222.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2102.83,7049.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2458.91,7213.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4078.80,7055.26");
    }

}

