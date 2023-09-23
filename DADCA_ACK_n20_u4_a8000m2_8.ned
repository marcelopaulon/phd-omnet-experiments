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

network DADCA_ACK_n20_u4_a8000m2_8
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
        @display("i=misc/sensor;p=682.17,7823.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1349.12,7872.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2338.90,7915.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4300.46,7999.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4850.24,7982.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5781.76,7956.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2111.88,7624.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3172.53,7551.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3716.65,7685.62");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4076.28,7604.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5410.16,7638.57");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=329.39,7386.80");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1259.22,7377.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2635.82,7433.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4481.01,7362.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4890.39,7450.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=830.83,7217.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1467.34,7011.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1789.72,7202.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2273.41,7207.23");
    }

}

