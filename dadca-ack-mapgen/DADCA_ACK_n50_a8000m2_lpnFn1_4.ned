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

network DADCA_ACK_n50_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=170.95,73.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=232.26,62.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=289.45,90.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=297.55,20.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=363.39,40.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=54.66,23.69");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=22.23,93.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=118.73,216.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=191.74,229.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=252.84,168.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=97.00,335.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=21.26,390.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=144.34,277.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=289.57,342.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=364.29,368.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=424.53,375.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=498.53,316.79");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=414.43,290.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=465.91,235.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=431.55,470.42");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=368.22,460.28");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=214.49,368.74");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=90.42,418.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=32.15,502.19");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=124.47,492.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=42.00,250.12");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=10.69,307.04");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=307.77,506.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=450.43,561.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=541.71,573.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=576.98,489.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=649.57,504.00");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=684.31,581.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=737.97,629.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=754.79,706.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=850.65,686.21");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=924.18,639.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=880.45,565.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=951.40,548.27");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=810.75,460.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=787.75,543.77");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=818.03,349.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=745.65,375.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=871.85,439.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=944.34,479.14");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=916.61,320.23");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=879.76,374.83");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=755.03,278.06");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=678.43,343.70");
    }

}

